#!/bin/bash

ps -ef | grep -E "(mainboard|cyber_launch|cyber_recorder|dreamview|data_cleaner)" | grep -v grep | awk '{print $2}' | xargs kill
sleep 2
ps -ef | grep -E "(mainboard|cyber_launch|cyber_recorder|dreamview|data_cleaner)" | grep -v grep | awk '{print $2}' | xargs kill -9
