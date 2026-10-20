#!/bin/bash

Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
echo -e "${Info} 这个是在 ROS 层面上地点记录的脚本, 仅仅用于 debug"
echo -e "${Info} 脚本运行过程任意时候可按 Ctrl+C 退出脚本"
echo -e ""
echo -e "${Info} 接下来请根据提示进行操作"

learn_mark() {
    local label=$1
    local mark=$2
    echo -e ""
    echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}${label}${Font_color_suffix}\c"
    echo && stty erase '^H' && read -p "       然后在这个界面内按回车..."
    rostopic pub /mark_nav std_msgs/String "data: 'learn ${mark}'" -1 >/dev/null
    echo -e "${Info} 地点名为 ${Yellow_background_prefix}${mark}${Font_color_suffix}"
}

learn_mark "分类区" "Classification_area"

learn_mark "中心九宫格北侧安全绕行点" "Transit_N"
learn_mark "中心九宫格东侧安全绕行点" "Transit_E"
learn_mark "中心九宫格南侧安全绕行点" "Transit_S"
learn_mark "中心九宫格西侧安全绕行点" "Transit_W"

learn_mark "A区收取区/蛋糕投放区" "Collection_A"
learn_mark "B区收取区/投放区" "Collection_B"
learn_mark "C区收取区/投放区" "Collection_C"
learn_mark "D区收取区/投放区" "Collection_D"

learn_mark "分拣区北" "Sorting_N"
learn_mark "分拣区西" "Sorting_W"
learn_mark "分拣区东（可用于补抓，若不用可录在安全空位）" "Sorting_E"
learn_mark "分拣区南（可用于补抓，若不用可录在安全空位）" "Sorting_S"


echo -e ""
echo -e "${Info} 是否标记完成,保存地图？ \c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'finish'" -1 >/dev/null
# 保存地图
CURRENTPATH=$(cd `dirname $0`; pwd)
gnome-terminal -x bash -c "rosrun map_server map_saver -f $CURRENTPATH/../config/test_map"
echo -e "${Info} 结束标记地点,已保存地图"

echo -e ""
echo -e "${Info} 地点记录已完成\c"
echo -e ""
echo && stty erase '^H' && read -p "现在可以按回车退出脚本..." 
