#!/usr/bin/env python3
"""
该模块提供了一个基于 `curses` 的终端用户界面（TUI），
用于控制一个ROS机器人进行路径记录和导航。
"""
import curses
import os
import threading

import rospy
from node import NAV_STATUS_RUNNING, Naver, PathRecorder

# --- 全局常量 ---
JUMP_TAB = {
    "RECORD": ["ADD", "INSERT", "DELETE", "IDLE","RECORD"],
    "ADD": ["ADD", "INSERT", "DELETE", "IDLE"],
    "INSERT": ["INSERT", "ADD", "DELETE", "IDLE"],
    "DELETE": ["DELETE", "ADD", "INSERT", "IDLE"],
    "NAV": ["NAV", "MVTO", "RECORD", "IDLE", "PAUSE"], # 【新增】: "PAUSE"
    "MVTO": ["NAV", "MVTO", "NAV", "IDLE", "PAUSE"], # 【新增】: "PAUSE"
    "IDLE": ["RECORD", "NAV"],
    "PAUSE": ["RESUME", "IDLE", "NAV", "MVTO"], # 【新增】: PAUSE状态下可恢复或中止
    "RESUME": ["PAUSE", "NAV", "MVTO", "RECORD", "IDLE"], # 【新增】: RESUME后状态会变回NAV，但为了逻辑完整性也加上
}

CMD_TIPS = {
    "RECORD": "listname:string",
    "ADD": "comment:string(optional)",
    "INSERT": "former_point:int | comment:string(optional)",
    "DELETE": "at_point:int",
    "NAV": "listname (or -listname for reverse)",
    "MVTO": "at_point:int",
    "IDLE": "",
    "PAUSE": "Pause the current navigation task", # 【新增】: 为新命令添加界面提示
    "RESUME": "Resume the paused navigation task", # 【新增】: 为新命令添加界面提示
}

MAX_CMD_ROWS = 25


def get_logo(height, width):
    """根据终端尺寸选择合适的ASCII Art Logo。"""
    __logo = r"""  ___       ______      _           _     _____ _____       _____ _____      _____ _____ 
 / _ \      | ___ \    | |         | |   |_   _|_   _|     |  __ |_   _|    |  _  / __  \
/ /_\ \ __ _| |_/ /___ | |__   ___ | |_    | |   | |       | |  \/ | |______| |/' `' / /'
|  _  |/ _` |    // _ \| '_ \ / _ \| __|   | |   | |       | | __  | |______|  /| | / /  
| | | | (_| | |\ | (_) | |_) | (_) | |_   _| |_ _| |_      | |_\ \ | |      \ |_/ ./ /___
\_| |_/\__, \_| \_\___/|_.__/ \___/ \__|  \___/ \___/       \____/ \_/       \___/\_____/
        __/ |                                                                            
       |___/                                                                             """

    __logo_small = r"""    _        ___     _        _     ___ ___        ___ _____     __ ___ 
   /_\  __ _| _ \___| |__ ___| |_  |_ _|_ _|      / __|_   ____ /  |_  )
  / _ \/ _` |   / _ | '_ / _ |  _|  | | | |      | (_ | | ||___| () / / 
 /_/ \_\__, |_|_\___|_.__\___/\__| |___|___|      \___| |_|     \__/___|
       |___/                                                            """

    __logo_text = "AgRobot II  GT-02"

    for logo in [__logo, __logo_small, __logo_text]:
        logo_lines = logo.split("\n")
        w = max([len(i.rstrip()) for i in logo_lines])
        h = len(logo_lines)
        if width > w and height // 2 > h:
            return logo
    return __logo_text


def setup(win):
    """初始化curses环境，设置颜色对。"""
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_CYAN)
    win.keypad(True)
    curses.echo()
    curses.curs_set(1)


def frame_welcome(win, height, width):
    """显示欢迎界面。"""
    win.nodelay(False)
    __wel_text = "DSLab AgRobot Terminal for GT-02"
    __wel_text2 = "<Press Any Key to Continue, 'q' to quit>"

    win.clear()
    start_y = height // 2
    start_x_wel_text = (width - len(__wel_text)) // 2
    start_x_wel_text2 = (width - len(__wel_text2)) // 2

    logo = get_logo(height, width)
    logo_lines = logo.split("\n")

    win.attron(curses.color_pair(1))
    for i, line in enumerate(logo_lines):
        start_x_l = (width - len(line)) // 2
        win.addstr(start_y - len(logo_lines) + i, start_x_l, line)
    win.attroff(curses.color_pair(1))

    win.attron(curses.color_pair(2) | curses.A_BOLD)
    win.addstr(start_y + 1, start_x_wel_text, __wel_text)
    win.attroff(curses.color_pair(2) | curses.A_BOLD)

    win.addstr(start_y + 3, start_x_wel_text2, __wel_text2)
    win.refresh()

    k = win.getch()
    if k == ord('q'):
        exit(0)

    win.clear()
    win.refresh()
    win.nodelay(True)


def main_loop(win, height, width):
    """主程序循环，处理UI绘制和用户命令。"""
    input_head = ' admin@GT-02 > '
    info_head = '│ Tips: '

    cmds = ["Welcome to AgRobot Terminal. Enter 'quit()' to exit."]

    base_dir = os.path.dirname(os.path.abspath(__file__))
    json_dir = os.path.join(base_dir, "paths_json")
    if not os.path.exists(json_dir):
        os.makedirs(json_dir)
        cmds.append(f"[SYSTEM] 创建路径存储目录: {json_dir}")

    exit_flag = False
    cur_stat = "IDLE"
    raw_input = ""
    warn = None

    recorder_naver = None
    nav_naver = None

    last_log_index = 0
    redraw_flag = True

    while not exit_flag:
        # --- 数据更新 ---
        active_naver = nav_naver or recorder_naver
        if active_naver and last_log_index < len(active_naver.log):
            new_logs = active_naver.log[last_log_index:]
            cmds.extend(new_logs)
            last_log_index = len(active_naver.log)
            redraw_flag = True
        elif nav_naver and nav_naver.status == NAV_STATUS_RUNNING:
            redraw_flag = True

        # --- 界面绘制 (仅在需要时) ---
        if redraw_flag:
            height, width = win.getmaxyx()
            max_cmd_rows = height - 4
            y_split, y_info, y_input = height - 3, height - 2, height - 1

            # 局部刷新，解决屏闪
            start_index = max(0, len(cmds) - max_cmd_rows)
            for i in range(max_cmd_rows):
                win.move(i, 0)
                win.clrtoeol()
                if i < len(cmds[start_index:]):
                    win.addstr(i, 0, str(cmds[start_index:][i])[:width - 1])

            win.move(y_split, 0)
            win.clrtoeol()
            win.addstr(y_split, 0, "┌" + '─' * (width - 2))

            win.move(y_info, 0)
            win.clrtoeol()
            info_body = f"Current: {cur_stat}, Legal: {' '.join([f'<{k}>' for k in JUMP_TAB[cur_stat]])}"
            
            # 【新增】: 在界面状态栏增加 [PAUSED] 状态显示
            if nav_naver and nav_naver.is_paused:
                info_body += " [PAUSED]"

            if nav_naver and nav_naver.status == NAV_STATUS_RUNNING:
                status_line = ""
                if nav_naver.current_position:
                    p = nav_naver.current_position
                    status_line += f" | Pos:({p['lat']:.5f},{p['lng']:.5f}) Yaw:{p['yaw']:.1f}°"
                if nav_naver.current_target_index is not None:
                    status_line += f" | Target: {nav_naver.current_target_index}/{len(nav_naver.points)}"
                info_body += status_line

            if warn:
                win.attron(curses.color_pair(2))
                win.addstr(y_info, 0, ("│ " + warn)[:width - 1])
                win.attroff(curses.color_pair(2))
                warn = None
            else:
                win.addstr(y_info, 0, (info_head + info_body)[:width - 1])

            win.move(y_input, 0)
            win.clrtoeol()
            win.attron(curses.color_pair(4))
            win.addstr(y_input, 0, input_head)
            win.attroff(curses.color_pair(4))
            win.addstr(y_input, len(input_head), raw_input)

            win.refresh()
            redraw_flag = False

        # --- 输入处理 ---
        try:
            ch = win.getch()
            if ch == -1:
                continue
        except curses.error:
            continue

        if ch in (curses.KEY_ENTER, 10, 13):
            input_command = raw_input.strip()
            raw_input = ""
            if not input_command:
                continue

            args = input_command.split()
            chstat, params = args[0].upper(), args[1:]
            redraw_flag = True

            if chstat == "QUIT()":
                if recorder_naver:
                    recorder_naver.save_points()
                if nav_naver:
                    nav_naver.stop_navigation()
                exit_flag = True
                continue

            if chstat not in JUMP_TAB.get(cur_stat, []):
                warn = f"Error: Command '{chstat}' not allowed from state '{cur_stat}'."
                continue

            cur_stat = chstat

            # --- 命令逻辑 ---
            if chstat == "RECORD":
                if not params:
                    warn = "Usage: RECORD <listname>"
                    continue
                listname = params[0]
                if recorder_naver and recorder_naver.is_alive():
                    recorder_naver.stop_event.set()
                if nav_naver and nav_naver.is_alive():
                    nav_naver.stop_navigation()
                recorder = PathRecorder(listname)
                recorder_naver = Naver(recorder, mode="record")
                recorder_naver.start()
                last_log_index = 0
                cmds.extend(recorder_naver.log)
                last_log_index = len(recorder_naver.log)

            # 【新增】: 在命令处理部分增加 PAUSE 和 RESUME 的逻辑
            elif chstat == "PAUSE":
                if nav_naver and nav_naver.is_alive():
                    nav_naver.pause_navigation()
                    cur_stat = "PAUSE"
                else:
                    warn = "No active navigation task to pause."

            elif chstat == "RESUME":
                if nav_naver and nav_naver.is_alive():
                    nav_naver.resume_navigation()
                    cur_stat = "NAV" # 恢复后状态回到NAV
                else:
                    warn = "No paused navigation task to resume."

            elif chstat in ("ADD", "INSERT", "DELETE"):
                if not recorder_naver:
                    warn = "Not in RECORD mode"
                    continue
                if chstat == "ADD":
                    recorder_naver.add_point(" ".join(params))
                elif chstat == "INSERT":
                    if not params or not params[0].isdigit():
                        warn = "Usage: INSERT <index> [comment]"
                        continue
                    recorder_naver.insert_point(int(params[0]), " ".join(params[1:]))
                elif chstat == "DELETE":
                    if not params or not params[0].isdigit():
                        warn = "Usage: DELETE <index>"
                        continue
                    recorder_naver.delete_point(int(params[0]))

            elif chstat in ("NAV", "MVTO"):
                listname, mvto_index, is_reverse = None, None, False
                if chstat == "NAV":
                    if not params:
                        warn = "Usage: NAV <listname> or -<listname>"
                        continue
                    filename = params[0]
                    is_reverse = filename.startswith('-')
                    listname = filename[1:] if is_reverse else filename
                else:  # MVTO
                    if not nav_naver:
                        warn = "Not in NAV mode. Use NAV first."
                        continue
                    if not params or not params[0].isdigit():
                        warn = "Usage: MVTO <point_index>"
                        continue
                    mvto_index = int(params[0])
                    listname = nav_naver.recorder.listname
                    if not (0 <= mvto_index < len(nav_naver.points)):
                        warn = f"Index {mvto_index} out of range"
                        continue

                json_path = os.path.join(json_dir, f"{listname}.json")
                if not os.path.exists(json_path):
                    available = [f[:-5] for f in os.listdir(json_dir) if f.endswith('.json')]
                    warn = f"Path file '{listname}.json' not found! Available: {', '.join(available) or 'None'}"
                    continue

                if nav_naver and nav_naver.is_alive():
                    nav_naver.stop_navigation()
                if recorder_naver and recorder_naver.is_alive():
                    recorder_naver.stop_event.set()

                dir_mode = "reverse" if is_reverse else "forward"
                recorder = PathRecorder(listname)
                nav_naver = Naver(recorder, mode="navigation", dir_mode=dir_mode, mvto_point=mvto_index)
                nav_naver.start()
                last_log_index = 0
                cmds.extend(nav_naver.log)
                last_log_index = len(nav_naver.log)

            elif chstat == "IDLE":
                if recorder_naver:
                    recorder_naver.save_points()
                    cmds.append(f"[SAVE] 路径 '{recorder_naver.recorder.listname}' 已保存")
                    recorder_naver = None
                if nav_naver:
                    nav_naver.stop_navigation()
                    cmds.extend(nav_naver.log[last_log_index:])
                    nav_naver = None
                cmds.append("[IDLE] 进入闲置状态")
                last_log_index = 0

        elif ch in (curses.KEY_BACKSPACE, 127):
            raw_input = raw_input[:-1]
            redraw_flag = True
        elif ch <= 255 and chr(ch).isprintable():
            raw_input += chr(ch)
            redraw_flag = True


def main(win):
    """主函数，包裹curses应用。"""
    rospy.init_node("agrobot_terminal_node", anonymous=True)
    setup(win)
    height, width = win.getmaxyx()
    try:
        frame_welcome(win, height, width)
        main_loop(win, height, width)
    except curses.error:
        # 忽略因调整窗口大小等原因产生的curses错误
        pass


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except KeyboardInterrupt:
        print("Program terminated by user.")
    except Exception as e:
        # 在退出curses后打印错误，避免屏幕混乱
        print(f"\nAn error occurred: {e}")