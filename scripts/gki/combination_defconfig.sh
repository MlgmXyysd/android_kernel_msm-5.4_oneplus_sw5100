#!/bin/bash
echo =======================================
echo =======start creat defconfig ==========
echo =======================================
echo $0
echo $1
echo $2
echo $3
echo $4
echo $5
config_path="$5/arch/$1/configs"
diff_config=$config_path"/vendor/"$4
qcom_config=$config_path"/"$3

echo config_path=$config_path
echo diff_config=$diff_config
echo qcom_config=$qcom_config

if [ ! -e $qcom_config ] ; then
	echo error: $qcom_config not exist
	exit -1
fi

if [ ! -e $diff_config ] ; then
	echo $diff_config not exist,creat a new diff config
	echo "#diff config for $2" >$diff_config
fi


# we only support 3 format item in diff file
# 1. not '#' head and include '='
# 2. '#' head and end of " is not set"
# 3. '-' head
# if this 3 format item in diff file ,and still in qcom defconfig
# we must delete the item in qcom defconfig

cp $qcom_config temp0

cat $diff_config | while read line
do
	if [[ ("$line" != '#'*) && ("$line" != '-'*) && ("$line" == *'='*) ]]
	then
		config_name=${line%%=*}
		echo $config_name
	elif [[ ("$line" == "# CONFIG"*) && ("$line" == *" is not set") ]]
	then
		config_name=${line:0:0-11}
		config_name=${config_name:2}
		echo not set: $config_name
	elif [[ "$line" == '-'* ]]
	then
		config_name=${line##*-}
		config_name=${config_name%%=*}
		echo del @: $config_name
	else
		continue
	fi

	sort_string=$config_name'='
	sed '/'"$sort_string"'/'d temp0 > temp1
	mv temp1 temp0

	sort_string=$config_name" is not set"
	sed '/'"$sort_string"'/'d temp0 > temp1
	mv temp1 temp0
done

mv temp0 $qcom_config
echo merge the $diff_config to $defconfig
cat $diff_config | while read line
do
	if [[ "$line" == '-'* ]]
	then
		continue
	fi
	echo $line
	echo $line >> $qcom_config
done

