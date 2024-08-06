// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *
 *  Bluetooth HCI UART driver
 *
 *  Copyright (C) 2002-2003  Fabrizio Gennari <fabrizio.gennari@philips.com>
 *  Copyright (C) 2004-2005  Marcel Holtmann <marcel@holtmann.org>
 */

#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/poll.h>

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/ioctl.h>
#include <linux/skbuff.h>
#include <linux/bitrev.h>
#include <asm/unaligned.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

static bool txcrc = true;
static bool hciextn = true;

#define BCSP_TXWINSIZE	1
#define BCSP_ACK_TIMEOUT	msecs_to_jiffies(250)
#define BCSP_SYNC_TIMEOUT	msecs_to_jiffies(100)

#define BCSP_ACK_PKT	0x05
#define BCSP_LE_PKT		0x06

/*
 * Maximum Three-wire packet:
 *     4 byte header + max value for 12-bit length + 2 bytes for CRC
 */
#define H5_MAX_LEN (4 + 0xfff + 2)

/* Convenience macros for reading Three-wire header values */
#define H5_HDR_SEQ(hdr)		((hdr)[0] & 0x07)
#define H5_HDR_ACK(hdr)		(((hdr)[0] >> 3) & 0x07)
#define H5_HDR_CRC(hdr)		(((hdr)[0] >> 6) & 0x01)
#define H5_HDR_RELIABLE(hdr)	(((hdr)[0] >> 7) & 0x01)
#define H5_HDR_PKT_TYPE(hdr)	((hdr)[1] & 0x0f)
#define H5_HDR_LEN(hdr)		((((hdr)[1] >> 4) & 0x0f) + ((hdr)[2] << 4))

#define SLIP_DELIMITER	0xc0
#define SLIP_ESC	0xdb
#define SLIP_ESC_DELIM	0xdc
#define SLIP_ESC_ESC	0xdd

static const u8 g_sync_req[4] = { 0xda, 0xdc, 0xed, 0xed };
static const u8 g_sync_rsp[4] = { 0xac, 0xaf, 0xef, 0xee };
static const u8 g_conf_req[4] = { 0xad, 0xef, 0xac, 0xed };
static const u8 g_conf_rsp[4] = { 0xde, 0xad, 0xd0, 0xd0 };

struct bcsp_struct {
	struct sk_buff_head unack;	/* Unack'ed packets queue */
	struct sk_buff_head rel;	/* Reliable packets queue */
	struct sk_buff_head unrel;	/* Unreliable packets queue */
	struct sk_buff_head pending_send;	/* Pending user data packets queue */

	unsigned long rx_count;
	struct	sk_buff *rx_skb;
	u8	rxseq_txack;		/* rxseq == txack. */
	u8	rxack;			/* Last packet sent by us that the peer ack'ed */
	u8	tx_win;		/* Sliding window size */
	struct	timer_list tbcsp;
	struct	hci_uart *hu;

	enum {
		BCSP_W4_PKT_DELIMITER,
		BCSP_W4_PKT_START,
		BCSP_W4_BCSP_HDR,
		BCSP_W4_DATA,
		BCSP_W4_CRC
	} rx_state;

	enum {
		BCSP_ESCSTATE_NOESC,
		BCSP_ESCSTATE_ESC
	} rx_esc_state;

	enum {
		H5_UNINITIALIZED,
		H5_INITIALIZED,
		H5_ACTIVE,
	} state;

	enum {
		H5_AWAKE,
		H5_SLEEPING,
		H5_WAKING_UP,
	} sleep;

	u8	use_crc;
	u16	message_crc;
	u8	txack_req;		/* Do we need to send ack's to the peer? */

	/* Reliable packet sequence number - used to assign seq to each rel pkt. */
	u8	msgq_txseq;
};

/* ---- BCSP CRC calculation ---- */

/* Table for calculating CRC for polynomial 0x1021, LSB processed first,
 * initial value 0xffff, bits shifted in reverse order.
 */

static const u16 crc_table[] = {
	0x0000, 0x1081, 0x2102, 0x3183,
	0x4204, 0x5285, 0x6306, 0x7387,
	0x8408, 0x9489, 0xa50a, 0xb58b,
	0xc60c, 0xd68d, 0xe70e, 0xf78f
};

/* Initialise the crc calculator */
#define BCSP_CRC_INIT(x) x = 0xffff

/* Update crc with next data byte
 *
 * Implementation note
 *     The data byte is treated as two nibbles.  The crc is generated
 *     in reverse, i.e., bits are fed into the register from the top.
 */
static void bcsp_crc_update(u16 *crc, u8 d)
{
	u16 reg = *crc;

	reg = (reg >> 4) ^ crc_table[(reg ^ d) & 0x000f];
	reg = (reg >> 4) ^ crc_table[(reg ^ (d >> 4)) & 0x000f];

	*crc = reg;
}

/* ---- BCSP core ---- */
//static void bcsp_reset_rx(struct bcsp_struct *bcsp)
//{
//	//if (bcsp->rx_skb) {
//	//	kfree_skb(bcsp->rx_skb);
//	//	bcsp->rx_skb = NULL;
//	//}
//
//	//clear_bit(H5_RX_ESC, &bcsp->flags);
//}

static void bcsp_link_control(struct hci_uart *hu, const void *data, size_t len)
{
	struct bcsp_struct *bcsp_struct = hu->priv;
	struct sk_buff *nskb;

	nskb = alloc_skb(len, GFP_ATOMIC);
	if (!nskb)
		return;

	hci_skb_pkt_type(nskb) = BCSP_LE_PKT;

	skb_put_data(nskb, data, len);

	skb_queue_tail(&bcsp_struct->unrel, nskb);
}

//static u8 bcsp_cfg_field(struct bcsp_struct *bcsp)
//{
//	/* Sliding window size (first 3 bits) */
//	return bcsp->tx_win & 0x07;
//}

static void bcsp_slip_msgdelim(struct sk_buff *skb)
{
	const char pkt_delim = 0xc0;

	skb_put_data(skb, &pkt_delim, 1);
}

static void bcsp_slip_one_byte(struct sk_buff *skb, u8 c)
{
	const char esc_c0[2] = { 0xdb, 0xdc };
	const char esc_db[2] = { 0xdb, 0xdd };

	switch (c) {
	case 0xc0:
		skb_put_data(skb, &esc_c0, 2);
		break;
	case 0xdb:
		skb_put_data(skb, &esc_db, 2);
		break;
	default:
		skb_put_data(skb, &c, 1);
	}
}

static int bcsp_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	struct bcsp_struct *bcsp = hu->priv;
	BT_DBG("%s enter", __func__);

	if (skb->len > 0xFFF) {
		BT_ERR("Packet too long");
		kfree_skb(skb);
		return 0;
	}

	if (bcsp->state != H5_ACTIVE) {
		BT_ERR("HCI data in non-active state, put in pendding queue");
		skb_queue_tail(&bcsp->pending_send, skb);
		BT_ERR("%s, pending queue size: %d", __func__, bcsp->pending_send.qlen);
		return 0;
	}

	switch (hci_skb_pkt_type(skb)) {
	case HCI_ACLDATA_PKT:
	case HCI_COMMAND_PKT:
		// BT_ERR("%s enter bcsp->rel size(%d)", __func__,skb_queue_len(&bcsp->rel));
		skb_queue_tail(&bcsp->rel, skb);
		break;

	case HCI_SCODATA_PKT:
		skb_queue_tail(&bcsp->unrel, skb);
		break;

	default:
		BT_ERR("Unknown packet type");
		kfree_skb(skb);
		break;
	}

	BT_DBG("%s, rel queue size: %d", __func__, bcsp->rel.qlen);
	BT_DBG("%s exit", __func__);
	return 0;
}

static struct sk_buff *bcsp_prepare_pkt(struct bcsp_struct *bcsp, u8 *data,
					int len, int pkt_type)
{
	struct sk_buff *nskb;
	u8 hdr[4], chan;
	u16 BCSP_CRC_INIT(bcsp_txmsg_crc);
	int rel, i;
	BT_DBG("%s enter", __func__);

	switch (pkt_type) {
	case HCI_ACLDATA_PKT:
		chan = 6;	/* BCSP ACL channel */
		rel = 1;	/* reliable channel */
		break;
	case HCI_COMMAND_PKT:
		chan = 5;	/* BCSP cmd/evt channel */
		rel = 1;	/* reliable channel */
		break;
	case HCI_SCODATA_PKT:
		chan = 7;	/* BCSP SCO channel */
		rel = 0;	/* unreliable channel */
		break;
	case BCSP_LE_PKT:
		chan = 1;	/* BCSP LE channel */
		rel = 0;	/* unreliable channel */
		break;
	case BCSP_ACK_PKT:
		chan = 0;	/* BCSP internal channel */
		rel = 0;	/* unreliable channel */
		break;
	default:
		BT_ERR("Unknown packet type");
		return NULL;
	}

	if (hciextn && chan == 5) {
		__le16 opcode = ((struct hci_command_hdr *)data)->opcode;

		/* Vendor specific commands */
		if (hci_opcode_ogf(__le16_to_cpu(opcode)) == 0x3f) {
			u8 desc = *(data + HCI_COMMAND_HDR_SIZE);

			if ((desc & 0xf0) == 0xc0) {
				data += HCI_COMMAND_HDR_SIZE + 1;
				len  -= HCI_COMMAND_HDR_SIZE + 1;
				chan = desc & 0x0f;
			}
		}
	}

	/* Max len of packet: (original len +4(bcsp hdr) +2(crc))*2
	 * (because bytes 0xc0 and 0xdb are escaped, worst case is
	 * when the packet is all made of 0xc0 and 0xdb :) )
	 * + 2 (0xc0 delimiters at start and end).
	 */

	nskb = alloc_skb((len + 6) * 2 + 2, GFP_ATOMIC);
	if (!nskb)
		return NULL;

	hci_skb_pkt_type(nskb) = pkt_type;

	bcsp_slip_msgdelim(nskb);

	hdr[0] = bcsp->rxseq_txack << 3;
	bcsp->txack_req = 0;
	BT_DBG("We request packet no %u to card", bcsp->rxseq_txack);

	if (rel) {
		hdr[0] |= 0x80 + bcsp->msgq_txseq;
		BT_DBG("Sending packet with seqno %u", bcsp->msgq_txseq);
		bcsp->msgq_txseq = (bcsp->msgq_txseq + 1) & 0x07;
	}

	if (bcsp->use_crc)
		hdr[0] |= 0x40;

	hdr[1] = ((len << 4) & 0xff) | chan;
	hdr[2] = len >> 4;
	hdr[3] = ~(hdr[0] + hdr[1] + hdr[2]);

	/* Put BCSP header */
	for (i = 0; i < 4; i++) {
		bcsp_slip_one_byte(nskb, hdr[i]);

		if (bcsp->use_crc)
			bcsp_crc_update(&bcsp_txmsg_crc, hdr[i]);
	}

	/* Put payload */
	for (i = 0; i < len; i++) {
		bcsp_slip_one_byte(nskb, data[i]);

		if (bcsp->use_crc)
			bcsp_crc_update(&bcsp_txmsg_crc, data[i]);
	}

	/* Put CRC */
	if (bcsp->use_crc) {
		bcsp_txmsg_crc = bitrev16(bcsp_txmsg_crc);
		bcsp_slip_one_byte(nskb, (u8)((bcsp_txmsg_crc >> 8) & 0x00ff));
		bcsp_slip_one_byte(nskb, (u8)(bcsp_txmsg_crc & 0x00ff));
	}

	bcsp_slip_msgdelim(nskb);
	BT_DBG("%s exit", __func__);
	return nskb;
}

/* This is a rewrite of pkt_avail in ABCSP */
static struct sk_buff *bcsp_dequeue(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;
	unsigned long flags;
	struct sk_buff *skb;

	/* First of all, check for unreliable messages in the queue,
	 * since they have priority
	 */

	BT_DBG("%s enter", __func__);
	skb = skb_dequeue(&bcsp->unrel);
	if (skb != NULL) {
		struct sk_buff *nskb;

		nskb = bcsp_prepare_pkt(bcsp, skb->data, skb->len,
					hci_skb_pkt_type(skb));
		if (nskb) {
			BT_DBG("%s exit", __func__);
			kfree_skb(skb);
			return nskb;
		} else {
			skb_queue_head(&bcsp->unrel, skb);
			BT_ERR("Could not dequeue pkt because alloc_skb failed");
		}
	}

	/* Now, try to send a reliable pkt. We can only send a
	 * reliable packet if the number of packets sent but not yet ack'ed
	 * is < than the winsize
	 */

	spin_lock_irqsave_nested(&bcsp->unack.lock, flags, SINGLE_DEPTH_NESTING);

	BT_DBG("%s: bcsp->unack.qlen:%d", __func__, bcsp->unack.qlen);
	BT_DBG("%s: bcsp->rel.qlen:%d", __func__, bcsp->rel.qlen);
	if (bcsp->unack.qlen < BCSP_TXWINSIZE) {
		skb = skb_dequeue(&bcsp->rel);
		if (skb != NULL) {
			struct sk_buff *nskb;

			nskb = bcsp_prepare_pkt(bcsp, skb->data, skb->len,
						hci_skb_pkt_type(skb));
			if (nskb) {
				__skb_queue_tail(&bcsp->unack, skb);
				BT_DBG("%s: restart timer", __func__);
				mod_timer(&bcsp->tbcsp, jiffies + BCSP_ACK_TIMEOUT);
				spin_unlock_irqrestore(&bcsp->unack.lock, flags);
				BT_DBG("%s exit", __func__);
				return nskb;
			} else {
				skb_queue_head(&bcsp->rel, skb);
				BT_ERR("Could not dequeue pkt because alloc_skb failed");
			}
		}
	}

	spin_unlock_irqrestore(&bcsp->unack.lock, flags);

	/* We could not send a reliable packet, either because there are
	 * none or because there are too many unack'ed pkts. Did we receive
	 * any packets we have not acknowledged yet ?
	 */

	if (bcsp->txack_req) {
		/* if so, craft an empty ACK pkt and send it on BCSP unreliable
		 * channel 0
		 */
		struct sk_buff *nskb = bcsp_prepare_pkt(bcsp, NULL, 0, BCSP_ACK_PKT);
		BT_DBG("%s: BCSP_ACK_PKT", __func__);
		BT_DBG("%s exit", __func__);
		return nskb;
	}

	BT_DBG("%s exit", __func__);
	/* We have nothing to send */
	return NULL;
}

static int bcsp_flush(struct hci_uart *hu)
{
	BT_DBG("hu %p", hu);
	return 0;
}

/* Remove ack'ed packets */
static void bcsp_pkt_cull(struct bcsp_struct *bcsp)
{
	struct sk_buff *skb, *tmp;
	unsigned long flags;
	int i, pkts_to_be_removed;
	u8 seqno;

	BT_DBG("%s enter", __func__);
	spin_lock_irqsave(&bcsp->unack.lock, flags);

	pkts_to_be_removed = skb_queue_len(&bcsp->unack);
	seqno = bcsp->msgq_txseq;

	while (pkts_to_be_removed) {
		if (bcsp->rxack == seqno)
			break;
		pkts_to_be_removed--;
		seqno = (seqno - 1) & 0x07;
	}

	if (bcsp->rxack != seqno)
		BT_ERR("Peer acked invalid packet");

	BT_DBG("Removing %u pkts out of %u, up to seqno %u",
	       pkts_to_be_removed, skb_queue_len(&bcsp->unack),
	       (seqno - 1) & 0x07);

	i = 0;
	skb_queue_walk_safe(&bcsp->unack, skb, tmp) {
		if (i >= pkts_to_be_removed)
			break;
		i++;

		__skb_unlink(skb, &bcsp->unack);
		kfree_skb(skb);
	}

	if (skb_queue_empty(&bcsp->unack)) {
		BT_DBG("%s: unack is empty, del timer", __func__);
		del_timer(&bcsp->tbcsp);
	}

	spin_unlock_irqrestore(&bcsp->unack.lock, flags);

	if (i != pkts_to_be_removed)
		BT_ERR("Removed only %u out of %u pkts", i, pkts_to_be_removed);
	BT_DBG("%s exit", __func__);

}

static void bcsp_link_ready(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;

	struct sk_buff *skb = NULL;
	while ((skb = skb_dequeue(&bcsp->pending_send))) {
		BT_ERR("%s, move pending queue to sending queue, pending size: %d", __func__, bcsp->pending_send.qlen);
		switch (hci_skb_pkt_type(skb)) {
		case HCI_ACLDATA_PKT:
		case HCI_COMMAND_PKT:
			skb_queue_tail(&bcsp->rel, skb);
			break;

		case HCI_SCODATA_PKT:
			skb_queue_tail(&bcsp->unrel, skb);
			break;

		default:
			BT_ERR("Unknown packet type");
			kfree_skb(skb);
			break;
		}
	}
}

static void bcsp_peer_reset(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;

	BT_ERR("Peer device has reset");

	bcsp->state = H5_UNINITIALIZED;

	del_timer(&bcsp->tbcsp);

	skb_queue_purge(&bcsp->rel);
	skb_queue_purge(&bcsp->unrel);
	skb_queue_purge(&bcsp->unack);
	skb_queue_purge(&bcsp->pending_send);

	bcsp->rxseq_txack = 0;
	bcsp->rxack = 0;

	bcsp->tx_win = BCSP_TXWINSIZE;
	bcsp->rx_state = BCSP_W4_PKT_DELIMITER;

	/* Send reset request to upper stack */
	hci_reset_dev(hu->hdev);
}


static void bcsp_handle_internal_rx(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;
	////const unsigned char sync_req[] = { 0x01, 0x7e };
	//u8 sync_req[4]     = { 0xda, 0xdc, 0xed, 0xed };
	//const unsigned char sync_rsp[] = { 0x02, 0x7d };
	//u8 conf_req[4]     = { 0xad, 0xef, 0xac, 0xed };
	////unsigned char conf_req[3] = { 0x03, 0xfc };
	////const unsigned char conf_rsp[] = { 0x04, 0x7b };
	//u8 conf_rsp_rsp[4] = { 0xde, 0xad, 0xd0, 0xd0 };
	//const unsigned char wakeup_req[] = { 0x05, 0xfa };
	//const unsigned char woken_req[] = { 0x06, 0xf9 };
	//const unsigned char sleep_req[] = { 0x07, 0x78 };
	//const unsigned char *hdr = bcsp->rx_skb->data;
	const unsigned char *data = &bcsp->rx_skb->data[4];

	BT_DBG("hdev name:%s", hu->hdev->name);

	//if (H5_HDR_PKT_TYPE(hdr) != BCSP_LE_PKT)
	//	return;

	//if (H5_HDR_LEN(hdr) < 2)
	//	return;

	//conf_req[2] = bcsp_cfg_field(bcsp);

	if (bcsp->rx_skb->data[1] >> 4 == 4 && bcsp->rx_skb->data[2] == 0 && 
			memcmp(data, g_sync_req, sizeof(g_sync_req)) == 0) {
		BT_DBG("link control, recv sync req");
		if (bcsp->state == H5_ACTIVE)
			bcsp_peer_reset(hu);
		bcsp_link_control(hu, g_sync_rsp, sizeof(g_sync_rsp));
		mod_timer(&bcsp->tbcsp, jiffies + BCSP_SYNC_TIMEOUT);
	} else if (bcsp->rx_skb->data[1] >> 4 == 4 && bcsp->rx_skb->data[2] == 0 && 
			memcmp(data, g_sync_rsp, sizeof(g_sync_rsp)) == 0) {
		if (bcsp->state == H5_ACTIVE)
			bcsp_peer_reset(hu);
		bcsp->state = H5_INITIALIZED;
		BT_DBG("link control, recv sync rsp");
		bcsp_link_control(hu, g_conf_req, sizeof(g_conf_req));
		mod_timer(&bcsp->tbcsp, jiffies + BCSP_SYNC_TIMEOUT);
	} else if (bcsp->rx_skb->data[1] >> 4 == 4 && bcsp->rx_skb->data[2] == 0 && 
			memcmp(data, g_conf_req, sizeof(g_sync_req)) == 0) {
		BT_DBG("link control, recv conf req");
		bcsp_link_control(hu, g_conf_rsp, sizeof(g_conf_rsp));
		bcsp_link_control(hu, g_conf_req, sizeof(g_conf_req));
		mod_timer(&bcsp->tbcsp, jiffies + BCSP_SYNC_TIMEOUT);
	} else if (bcsp->rx_skb->data[1] >> 4 == 4 && bcsp->rx_skb->data[2] == 0 && 
			memcmp(data, g_conf_rsp, sizeof(g_conf_rsp)) == 0) {
		//if (H5_HDR_LEN(hdr) > 2)
		//	bcsp->tx_win = (data[2] & 0x07);
		//BT_DBG("Three-wire init complete. tx_win %u", bcsp->tx_win);
		bcsp->state = H5_ACTIVE;
		//need to wake tx worker to send data
		//hci_uart_init_ready(hu);
		BT_DBG("link control, recv conf rsp");
		bcsp_link_ready(hu);
		//return;
	//} else if (memcmp(data, sleep_req, 2) == 0) {
	//	BT_DBG("Peer went to sleep");
	//	bcsp->sleep = H5_SLEEPING;
	//	return;
	//} else if (memcmp(data, woken_req, 2) == 0) {
	//	BT_DBG("Peer woke up");
	//	bcsp->sleep = H5_AWAKE;
	//} else if (memcmp(data, wakeup_req, 2) == 0) {
	//	BT_DBG("Peer requested wakeup");
	//	bcsp_link_control(hu, woken_req, 2);
	//	bcsp->sleep = H5_AWAKE;
	} else {
		BT_DBG("Link Control: 0x%02hhx 0x%02hhx", data[0], data[1]);
		return;
	}

	hci_uart_tx_wakeup(hu);
}

/* Handle BCSP link-establishment packets. When we
 * detect a "sync" packet, symptom that the BT module has reset,
 * we do nothing :) (yet)
 */
static void bcsp_handle_le_pkt(struct hci_uart *hu)
{
#if 1
	bcsp_handle_internal_rx(hu);
#else
	struct bcsp_struct *bcsp = hu->priv;
	u8 conf_pkt[4]     = { 0xad, 0xef, 0xac, 0xed };
	u8 conf_rsp_pkt[4] = { 0xde, 0xad, 0xd0, 0xd0 };
	u8 sync_pkt[4]     = { 0xda, 0xdc, 0xed, 0xed };

	/* spot "conf" pkts and reply with a "conf rsp" pkt */
	if (bcsp->rx_skb->data[1] >> 4 == 4 && bcsp->rx_skb->data[2] == 0 &&
	    !memcmp(&bcsp->rx_skb->data[4], conf_pkt, 4)) {
		struct sk_buff *nskb = alloc_skb(4, GFP_ATOMIC);

		BT_DBG("Found a LE conf pkt");
		if (!nskb)
			return;
		skb_put_data(nskb, conf_rsp_pkt, 4);
		hci_skb_pkt_type(nskb) = BCSP_LE_PKT;

		skb_queue_head(&bcsp->unrel, nskb);
		hci_uart_tx_wakeup(hu);
	}
	/* Spot "sync" pkts. If we find one...disaster! */
	else if (bcsp->rx_skb->data[1] >> 4 == 4 && bcsp->rx_skb->data[2] == 0 &&
		 !memcmp(&bcsp->rx_skb->data[4], sync_pkt, 4)) {
		BT_ERR("Found a LE sync pkt, card has reset");
	}
#endif
}

static inline void bcsp_unslip_one_byte(struct bcsp_struct *bcsp, unsigned char byte)
{
	const u8 c0 = 0xc0, db = 0xdb;

	switch (bcsp->rx_esc_state) {
	case BCSP_ESCSTATE_NOESC:
		switch (byte) {
		case 0xdb:
			bcsp->rx_esc_state = BCSP_ESCSTATE_ESC;
			break;
		default:
			skb_put_data(bcsp->rx_skb, &byte, 1);
			if ((bcsp->rx_skb->data[0] & 0x40) != 0 &&
			    bcsp->rx_state != BCSP_W4_CRC)
				bcsp_crc_update(&bcsp->message_crc, byte);
			bcsp->rx_count--;
		}
		break;

	case BCSP_ESCSTATE_ESC:
		switch (byte) {
		case 0xdc:
			skb_put_data(bcsp->rx_skb, &c0, 1);
			if ((bcsp->rx_skb->data[0] & 0x40) != 0 &&
			    bcsp->rx_state != BCSP_W4_CRC)
				bcsp_crc_update(&bcsp->message_crc, 0xc0);
			bcsp->rx_esc_state = BCSP_ESCSTATE_NOESC;
			bcsp->rx_count--;
			break;

		case 0xdd:
			skb_put_data(bcsp->rx_skb, &db, 1);
			if ((bcsp->rx_skb->data[0] & 0x40) != 0 &&
			    bcsp->rx_state != BCSP_W4_CRC)
				bcsp_crc_update(&bcsp->message_crc, 0xdb);
			bcsp->rx_esc_state = BCSP_ESCSTATE_NOESC;
			bcsp->rx_count--;
			break;

		default:
			BT_ERR("Invalid byte %02x after esc byte", byte);
			kfree_skb(bcsp->rx_skb);
			bcsp->rx_skb = NULL;
			bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
			bcsp->rx_count = 0;
		}
	}
}

static void bcsp_complete_rx_pkt(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;
	int pass_up = 0;

	BT_DBG("%s enter", __func__);
	if (bcsp->rx_skb->data[0] & 0x80) {	/* reliable pkt */
		BT_DBG("Received seqno %u from card", bcsp->rxseq_txack);

		/* check the rx sequence number is as expected */
		if ((bcsp->rx_skb->data[0] & 0x07) == bcsp->rxseq_txack) {
			bcsp->rxseq_txack++;
			bcsp->rxseq_txack %= 0x8;
		} else {
			/* handle re-transmitted packet or
			 * when packet was missed
			 */
			BT_ERR("Out-of-order packet arrived, got %u expected %u",
			       bcsp->rx_skb->data[0] & 0x07, bcsp->rxseq_txack);

			/* do not process out-of-order packet payload */
			pass_up = 2;
		}

		/* send current txack value to all received reliable packets */
		bcsp->txack_req = 1;

		/* If needed, transmit an ack pkt */
		hci_uart_tx_wakeup(hu);
	} 

	bcsp->rxack = (bcsp->rx_skb->data[0] >> 3) & 0x07;
	BT_DBG("Request for pkt %u from card", bcsp->rxack);

	/* handle received ACK indications,
	 * including those from out-of-order packets
	 */
	bcsp_pkt_cull(bcsp);

	if(!skb_queue_empty(&bcsp->unack) || !skb_queue_empty(&bcsp->rel) ){
		BT_DBG("%s, rel queue size: %d unack queue size:%d", __func__, bcsp->rel.qlen, bcsp->unack.qlen);
		hci_uart_tx_wakeup(hu);
	}

	if (pass_up != 2) {
		if ((bcsp->rx_skb->data[1] & 0x0f) == 6 &&
		    (bcsp->rx_skb->data[0] & 0x80)) {
			hci_skb_pkt_type(bcsp->rx_skb) = HCI_ACLDATA_PKT;
			pass_up = 1;
		} else if ((bcsp->rx_skb->data[1] & 0x0f) == 5 &&
			   (bcsp->rx_skb->data[0] & 0x80)) {
			hci_skb_pkt_type(bcsp->rx_skb) = HCI_EVENT_PKT;
			pass_up = 1;
		} else if ((bcsp->rx_skb->data[1] & 0x0f) == 7) {
			hci_skb_pkt_type(bcsp->rx_skb) = HCI_SCODATA_PKT;
			pass_up = 1;
		} else if ((bcsp->rx_skb->data[1] & 0x0f) == 1 &&
			   !(bcsp->rx_skb->data[0] & 0x80)) {
			bcsp_handle_le_pkt(hu);
			pass_up = 0;
		} else {
			pass_up = 0;
		}
	}

	if (pass_up == 0) {
		struct hci_event_hdr hdr;
		u8 desc = (bcsp->rx_skb->data[1] & 0x0f);

		if (desc != 0 && desc != 1) {
			if (hciextn) {
				desc |= 0xc0;
				skb_pull(bcsp->rx_skb, 4);
				memcpy(skb_push(bcsp->rx_skb, 1), &desc, 1);

				hdr.evt = 0xff;
				hdr.plen = bcsp->rx_skb->len;
				memcpy(skb_push(bcsp->rx_skb, HCI_EVENT_HDR_SIZE), &hdr, HCI_EVENT_HDR_SIZE);
				hci_skb_pkt_type(bcsp->rx_skb) = HCI_EVENT_PKT;

				hci_recv_frame(hu->hdev, bcsp->rx_skb);
			} else {
				BT_ERR("Packet for unknown channel (%u %s)",
				       bcsp->rx_skb->data[1] & 0x0f,
				       bcsp->rx_skb->data[0] & 0x80 ?
				       "reliable" : "unreliable");
				kfree_skb(bcsp->rx_skb);
			}
		} else
			kfree_skb(bcsp->rx_skb);
	} else if (pass_up == 1) {
		/* Pull out BCSP hdr */
		skb_pull(bcsp->rx_skb, 4);

		hci_recv_frame(hu->hdev, bcsp->rx_skb);
	} else {
		/* ignore packet payload of already ACKed re-transmitted
		 * packets or when a packet was missed in the BCSP window
		 */
		kfree_skb(bcsp->rx_skb);
	}

	bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
	bcsp->rx_skb = NULL;
	BT_DBG("%s exit", __func__);
}

static u16 bscp_get_crc(struct bcsp_struct *bcsp)
{
	return get_unaligned_be16(&bcsp->rx_skb->data[bcsp->rx_skb->len - 2]);
}

/* Recv data */
static int bcsp_recv(struct hci_uart *hu, const void *data, int count)
{
	struct bcsp_struct *bcsp = hu->priv;
	const unsigned char *ptr;

	BT_DBG("hu %p count %d rx_state %d rx_count %ld",
	       hu, count, bcsp->rx_state, bcsp->rx_count);

	ptr = data;
	while (count) {
		if (bcsp->rx_count) {
			if (*ptr == 0xc0) {
				BT_ERR("Short BCSP packet");
				kfree_skb(bcsp->rx_skb);
				bcsp->rx_skb = NULL;
				bcsp->rx_state = BCSP_W4_PKT_START;
				bcsp->rx_count = 0;
			} else
				bcsp_unslip_one_byte(bcsp, *ptr);

			ptr++; count--;
			continue;
		}

		switch (bcsp->rx_state) {
		case BCSP_W4_BCSP_HDR:
			if ((0xff & (u8)~(bcsp->rx_skb->data[0] + bcsp->rx_skb->data[1] +
			    bcsp->rx_skb->data[2])) != bcsp->rx_skb->data[3]) {
				BT_ERR("Error in BCSP hdr checksum");
				kfree_skb(bcsp->rx_skb);
				bcsp->rx_skb = NULL;
				bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
				bcsp->rx_count = 0;
				continue;
			}
			bcsp->rx_state = BCSP_W4_DATA;
			bcsp->rx_count = (bcsp->rx_skb->data[1] >> 4) +
					(bcsp->rx_skb->data[2] << 4);	/* May be 0 */
			continue;

		case BCSP_W4_DATA:
			if (bcsp->rx_skb->data[0] & 0x40) {	/* pkt with crc */
				bcsp->rx_state = BCSP_W4_CRC;
				bcsp->rx_count = 2;
			} else
				bcsp_complete_rx_pkt(hu);
			continue;

		case BCSP_W4_CRC:
			if (bitrev16(bcsp->message_crc) != bscp_get_crc(bcsp)) {
				BT_ERR("Checksum failed: computed %04x received %04x",
				       bitrev16(bcsp->message_crc),
				       bscp_get_crc(bcsp));

				kfree_skb(bcsp->rx_skb);
				bcsp->rx_skb = NULL;
				bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
				bcsp->rx_count = 0;
				continue;
			}
			skb_trim(bcsp->rx_skb, bcsp->rx_skb->len - 2);
			bcsp_complete_rx_pkt(hu);
			continue;

		case BCSP_W4_PKT_DELIMITER:
			switch (*ptr) {
			case 0xc0:
				bcsp->rx_state = BCSP_W4_PKT_START;
				break;
			default:
				/*BT_ERR("Ignoring byte %02x", *ptr);*/
				break;
			}
			ptr++; count--;
			break;

		case BCSP_W4_PKT_START:
			switch (*ptr) {
			case 0xc0:
				ptr++; count--;
				break;

			default:
				bcsp->rx_state = BCSP_W4_BCSP_HDR;
				bcsp->rx_count = 4;
				bcsp->rx_esc_state = BCSP_ESCSTATE_NOESC;
				BCSP_CRC_INIT(bcsp->message_crc);

				/* Do not increment ptr or decrement count
				 * Allocate packet. Max len of a BCSP pkt=
				 * 0xFFF (payload) +4 (header) +2 (crc)
				 */

				bcsp->rx_skb = bt_skb_alloc(0x1005, GFP_ATOMIC);
				if (!bcsp->rx_skb) {
					BT_ERR("Can't allocate mem for new packet");
					bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
					bcsp->rx_count = 0;
					return 0;
				}
				break;
			}
			break;
		}
	}
	return count;
}

	/* Arrange to retransmit all messages in the relq. */
static void bcsp_timed_event(struct timer_list *t)
{
	//const unsigned char sync_req[] = { 0x01, 0x7e };
	//unsigned char conf_req[3] = { 0x03, 0xfc };

	struct bcsp_struct *bcsp = from_timer(bcsp, t, tbcsp);
	struct hci_uart *hu = bcsp->hu;
	struct sk_buff *skb;
	unsigned long flags;

	BT_DBG("%s enter", __func__);

	if (bcsp->state == H5_UNINITIALIZED) {
		BT_DBG("%s, resend sync req", __func__);
		bcsp_link_control(hu, g_sync_req, sizeof(g_sync_req));
	}

	if (bcsp->state == H5_INITIALIZED) {
		//conf_req[2] = bcsp_cfg_field(bcsp);
		BT_DBG("%s, resend conf req", __func__);
		bcsp_link_control(hu, g_conf_req, sizeof(g_conf_req));
	}

	if (bcsp->state != H5_ACTIVE) {
		mod_timer(&bcsp->tbcsp, jiffies + BCSP_SYNC_TIMEOUT);
		goto wakeup;
	}

	//if (bcsp->sleep != H5_AWAKE) {
	//	bcsp->sleep = H5_SLEEPING;
	//	goto wakeup;
	//}

	BT_DBG("hu %p retransmitting %u pkts", hu, bcsp->unack.qlen);

	spin_lock_irqsave_nested(&bcsp->unack.lock, flags, SINGLE_DEPTH_NESTING);

	while ((skb = __skb_dequeue_tail(&bcsp->unack)) != NULL) {
		bcsp->msgq_txseq = (bcsp->msgq_txseq - 1) & 0x07;
		BT_ERR("ReSending packet with seqno %u", bcsp->msgq_txseq);
		skb_queue_head(&bcsp->rel, skb);
	}
	spin_unlock_irqrestore(&bcsp->unack.lock, flags);

wakeup:
	hci_uart_tx_wakeup(hu);
	BT_DBG("%s exit", __func__);
}

static int bcsp_open(struct hci_uart *hu)
{
	//const unsigned char sync[] = { 0x01, 0x7e };
	struct bcsp_struct *bcsp;

	BT_DBG("hu %p", hu);

	bcsp = kzalloc(sizeof(*bcsp), GFP_KERNEL);
	if (!bcsp)
		return -ENOMEM;

	hu->priv = bcsp;
	bcsp->hu = hu;
	skb_queue_head_init(&bcsp->unack);
	skb_queue_head_init(&bcsp->rel);
	skb_queue_head_init(&bcsp->unrel);
	skb_queue_head_init(&bcsp->pending_send);


	timer_setup(&bcsp->tbcsp, bcsp_timed_event, 0);

	bcsp->tx_win = BCSP_TXWINSIZE;
	bcsp->rx_state = BCSP_W4_PKT_DELIMITER;
	bcsp->state = H5_UNINITIALIZED;

	if (txcrc)
		bcsp->use_crc = 1;

	/* Send initial sync request */
	BT_ERR("%s, start sending sync timer", __func__);
	//bcsp_link_control(hu, g_sync_req, sizeof(g_sync_req));
	//hci_uart_tx_wakeup(hu);
	mod_timer(&bcsp->tbcsp, jiffies + BCSP_SYNC_TIMEOUT);

	return 0;
}

static int bcsp_close(struct hci_uart *hu)
{
	struct bcsp_struct *bcsp = hu->priv;

	bcsp->state = H5_UNINITIALIZED;
	del_timer_sync(&bcsp->tbcsp);

	hu->priv = NULL;

	BT_DBG("%s, hu %p", __func__, hu);

	skb_queue_purge(&bcsp->unack);
	skb_queue_purge(&bcsp->rel);
	skb_queue_purge(&bcsp->unrel);
	skb_queue_purge(&bcsp->pending_send);

	if (bcsp->rx_skb) {
		kfree_skb(bcsp->rx_skb);
		bcsp->rx_skb = NULL;
	}

	kfree(bcsp);
	return 0;
}

static const struct hci_uart_proto bcsp = {
	.id		= HCI_UART_BCSP,
	.name		= "BCSP",
	.open		= bcsp_open,
	.close		= bcsp_close,
	.enqueue	= bcsp_enqueue,
	.dequeue	= bcsp_dequeue,
	.recv		= bcsp_recv,
	.flush		= bcsp_flush
};

int __init bcsp_init(void)
{
	return hci_uart_register_proto(&bcsp);
}

int __exit bcsp_deinit(void)
{
	return hci_uart_unregister_proto(&bcsp);
}

module_param(txcrc, bool, 0644);
MODULE_PARM_DESC(txcrc, "Transmit CRC with every BCSP packet");

module_param(hciextn, bool, 0644);
MODULE_PARM_DESC(hciextn, "Convert HCI Extensions into BCSP packets");
