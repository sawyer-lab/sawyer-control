#!/usr/bin/env python3
"""Nudge one Sawyer joint at a time from a terminal."""
import curses

from sawyer_control import ControlMode, JointCommandSample, SawyerRobotClient

STEP_RAD = 0.05


def _draw(screen, selected, target, message):
    screen.erase()
    screen.addstr(0, 0, 'Sawyer keyboard joint control')
    screen.addstr(2, 0, '1–7 select J0–J6 · left/right nudge · q quit')
    screen.addstr(4, 0, f'Selected: J{selected}     step: {STEP_RAD:.3f} rad')
    for index, value in enumerate(target):
        marker = '>' if index == selected else ' '
        screen.addstr(6 + index, 0, f'{marker} J{index}: {value:+.4f} rad')
    screen.addstr(15, 0, message)
    screen.refresh()


def _run(screen):
    curses.curs_set(0)
    screen.keypad(True)
    with SawyerRobotClient.connect() as robot:
        target = list(robot.get_state().positions.values)
        selected = 0
        message = 'Read current joint positions. Robot lifecycle is unchanged.'
        while True:
            _draw(screen, selected, target, message)
            key = screen.getch()
            if key in (ord('q'), ord('Q')):
                return
            if ord('1') <= key <= ord('7'):
                selected = key - ord('1')
                message = f'Selected J{selected}.'
                continue
            if key == curses.KEY_LEFT:
                direction = -1
            elif key == curses.KEY_RIGHT:
                direction = 1
            else:
                message = 'Use 1–7, left/right, or q.'
                continue
            target[selected] += direction * STEP_RAD
            try:
                robot.command(JointCommandSample(position=target), ControlMode.POSITION)
                message = f'Sent J{selected} target: {target[selected]:+.4f} rad.'
            except Exception as error:
                target = list(robot.get_state().positions.values)
                message = f'Command failed: {error}'


def main():
    curses.wrapper(_run)


if __name__ == '__main__':
    main()
