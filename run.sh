#!/bin/bash
#功能：编译运行

#编译
cd ./code/build
cmake ../
make
cd ../../

echo $#  #sh参数个数
echo $@  #所有参数


# 执行./run.sh ：执行所有地图，求平均分
if [ $# -eq 0 ]; then
    # 设置循环次数和初始累加值
    iterations=10
    sum=0

    # 创建空数组来存储分数
    scores=()

    # 循环执行命令并累加分数值
    for i in $(seq 1 $iterations); do
        score=$(./LinuxRelease/PreliminaryJudge ./code/build/main -m ./LinuxRelease/maps/map$i.txt -l WARN -f 15 | grep -o '"score":[0-9]*' | awk -F: '{print $2}')
        sum=$((sum + score))
        scores+=($score)  # 将分数添加到数组中
        echo "map$i : $score"
    done

    # 计算平均值
    average=$((sum / iterations))

    echo "平均分数为: $average"

# 执行./run.sh [1,10] ：执行指定地图 
else
    ./LinuxRelease/PreliminaryJudge ./code/build/main -m ./LinuxRelease/maps/map$1.txt -l NONE -f 15
fi



# Usage:
#    ./PreliminaryJudge [options...] <player's program>
# Options:
#    -f  Time for player to process per frame in ms. 0 means infinite time. default: 15
#    -m  Specify the map file.
#    -r  Specify the replay file to output. DO NOT CONTAIN FOLDER! default: '%Y-%m-%d.%H.%M.%S.rep'
#    -s  Specify the random seed. Integer in [0,2^31).
#    -l  Specify the log level:[DBG|INFO|WARN|ERR|ASSERT|NONE], defalut:INFO
#    -d  Specify the output file. Judge will write all output to this file.
#    -h  Display this information
# Sample:
#    ./PreliminaryJudge -f 0 -m .\map.txt -s %Y-%m-%d.%H.%M.%S.rep -l INFO -d .\output.txt "python main.py"

#24.03.07

