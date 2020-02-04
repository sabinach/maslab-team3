#!/usr/bin/env python

# Helper script: Set gamepad mappings (different controllers may have different mappings)

def set_gamepad_mappings(msg):
       gamepad_mappings = { "button_horizontal":    msg.axes[6],
                            "button_vertical":      msg.axes[7],
                            "left_joy_horizontal":  msg.axes[0],
                            "left_joy_vertical":    msg.axes[1],
                            "right_joy_horizontal": msg.axes[3],
                            "right_joy_vertical":   msg.axes[4],
                            "button_lt":            msg.axes[2],
                            "button_rt":            msg.axes[5],
                            "button_a":             msg.buttons[0],
                            "button_b":             msg.buttons[1],
                            "button_x":             msg.buttons[2],
                            "button_y":             msg.buttons[3],
                            "button_lb":            msg.buttons[4],
                            "button_rb":            msg.buttons[5],
                            "button_back":          msg.buttons[6],
                            "button_start":         msg.buttons[7],
                            "button_center":        msg.buttons[8]
                            }
       return gamepad_mappings