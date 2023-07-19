#!/bin/bash

# 获取要运行的命令和持续时间
command="$1"
echo "command: $command"
duration=20

# 启动进程
$command &

# 获取进程ID
pid=$!

echo "子进程ID: $pid，请等待 $duration 秒..."

# 等待一段时间
sleep $duration

# 发送SIGINT信号给进程
kill -2 $pid

# 等待进程退出
wait $pid

# 输出进程的返回码
echo "子进程返回码: $?"
