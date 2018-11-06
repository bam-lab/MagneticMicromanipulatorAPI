#! /usr/bin/env python
#  -*- coding: utf-8 -*-
#
# GUI module generated by PAGE version 4.17
# In conjunction with Tcl version 8.6
#    Nov 05, 2018 02:38:38 AM EST  platform: Linux

import sys

try:
    from Tkinter import *
except ImportError:
    from tkinter import *

try:
    import ttk
    py3 = False
except ImportError:
    import tkinter.ttk as ttk
    py3 = True

import gui_support

def vp_start_gui():
    '''Starting point when module is the main routine.'''
    global val, w, root
    root = Tk()
    gui_support.set_Tk_var()
    top = GUI (root)
    gui_support.init(root, top)
    root.mainloop()

w = None
def create_GUI(root, *args, **kwargs):
    '''Starting point when module is imported by another program.'''
    global w, w_win, rt
    rt = root
    w = Toplevel (root)
    gui_support.set_Tk_var()
    top = GUI (w)
    gui_support.init(w, top, *args, **kwargs)
    return (w, top)

def destroy_GUI():
    global w
    w.destroy()
    w = None


class GUI:
    def __init__(self, top=None):
        '''This class configures and populates the toplevel window.
           top is the toplevel containing window.'''
        _bgcolor = '#d9d9d9'  # X11 color: 'gray85'
        _fgcolor = '#000000'  # X11 color: 'black'
        _compcolor = '#d9d9d9' # X11 color: 'gray85'
        _ana1color = '#d9d9d9' # X11 color: 'gray85' 
        _ana2color = '#d9d9d9' # X11 color: 'gray85' 
        font9 = "-family Verdana -size 12 -weight normal -slant roman "  \
            "-underline 0 -overstrike 0"
        self.style = ttk.Style()
        if sys.platform == "win32":
            self.style.theme_use('winnative')
        self.style.configure('.',background=_bgcolor)
        self.style.configure('.',foreground=_fgcolor)
        self.style.configure('.',font="TkDefaultFont")
        self.style.map('.',background=
            [('selected', _compcolor), ('active',_ana2color)])

        top.geometry("758x629+162+74")
        top.title("GUI")
        top.configure(highlightcolor="black")



        self.menubar = Menu(top,font="TkMenuFont",bg=_bgcolor,fg=_fgcolor)
        top.configure(menu = self.menubar)

        def demagnetization():
            self.console_output.insert(1.0, "Demagnetization in progres...\n")
            #TODO demagnetization call
            self.console_output.insert(1.0, "Demagnetization complete\n")

        def status_refresh():
            #TODO Refresh all value labels (mm.getstatus, mm.getposition)
            self.console_output.insert(1.0, "Status page refreshed\n")

        def go_to_position():
            self.Button_gtp.configure(state="disabled")
            x = gui_support.gtp_x.get()
            y = gui_support.gtp_y.get()
            z = gui_support.gtp_z.get()
            self.console_output.insert(1.0, "Moving to "+x+"x "+y+"y "+z+"z...\n")
            #TODO call manipulator instance go to position function
            self.console_output.insert(1.0, "Moving complete\n")
            self.Button_gtp.configure(state="normal")

        def step_x():
            x = gui_support.step_x.get()
            self.console_output.insert(1.0, "Moving by "+x+"x\n")
            #TODO call manipulator instance go to position function
            self.console_output.insert(1.0, "Moving complete\n")

        def step_y():
            y = gui_support.step_y.get()
            self.console_output.insert(1.0, "Moving by "+y+"y\n")
            #TODO call manipulator instance go to position function
            self.console_output.insert(1.0, "Moving complete\n")

        def step_z():
            z = gui_support.step_z.get()
            self.console_output.insert(1.0, "Moving by "+z+"z\n")
            #TODO call manipulator instance go to position function
            self.console_output.insert(1.0, "Moving complete\n")

        def change_velocity():
            vel = gui_support.velocity.get()
            self.console_output.insert(1.0, "Changing velocity to " + vel + "um/s\n")
            #TODO mm.change velocity
            self.console_output.insert(1.0, "Changed velocity\n")

        def disable_interface():
            #TODO disable all user interaction (buttons). Typically used when waiting for a mm move to finish
            '''does this work'''


        self.MM_Frame = Frame(top)
        self.MM_Frame.place(relx=0.0, rely=0.0, relheight=0.469, relwidth=0.534)
        self.MM_Frame.configure(relief=RAISED)
        self.MM_Frame.configure(borderwidth="2")
        self.MM_Frame.configure(relief=RAISED)
        self.MM_Frame.configure(width=405)

        self.Entry_gtp_x = Entry(self.MM_Frame)
        self.Entry_gtp_x.place(relx=0.049, rely=0.136, height=20, relwidth=0.114)

        self.Entry_gtp_x.configure(background="white")
        self.Entry_gtp_x.configure(font="TkFixedFont")
        self.Entry_gtp_x.configure(selectbackground="#c4c4c4")
        self.Entry_gtp_x.configure(textvariable=gui_support.gtp_x)

        self.Label_Manipulator = Label(self.MM_Frame)
        self.Label_Manipulator.place(relx=0.049, rely=0.034, height=28
                                     , width=136)
        self.Label_Manipulator.configure(activebackground="#f9f9f9")
        self.Label_Manipulator.configure(font=font9)
        self.Label_Manipulator.configure(text='''Micromanipulator''')

        self.Entry_gtp_y = Entry(self.MM_Frame)
        self.Entry_gtp_y.place(relx=0.173, rely=0.136, height=20, relwidth=0.114)

        self.Entry_gtp_y.configure(background="white")
        self.Entry_gtp_y.configure(font="TkFixedFont")
        self.Entry_gtp_y.configure(selectbackground="#c4c4c4")
        self.Entry_gtp_y.configure(textvariable=gui_support.gtp_y)

        self.Entry_gtp_z = Entry(self.MM_Frame)
        self.Entry_gtp_z.place(relx=0.296, rely=0.136, height=20, relwidth=0.114)

        self.Entry_gtp_z.configure(background="white")
        self.Entry_gtp_z.configure(font="TkFixedFont")
        self.Entry_gtp_z.configure(selectbackground="#c4c4c4")
        self.Entry_gtp_z.configure(textvariable=gui_support.gtp_z)

        self.Button_gtp = Button(self.MM_Frame, command=lambda:go_to_position())
        self.Button_gtp.place(relx=0.049, rely=0.203, height=26, width=149)
        self.Button_gtp.configure(activebackground="#d9d9d9")
        self.Button_gtp.configure(text='''Go To Position (um)''')

        self.Button_step_x = Button(self.MM_Frame, command=lambda:step_x())
        self.Button_step_x.place(relx=0.198, rely=0.508, height=26, width=32)
        self.Button_step_x.configure(activebackground="#d9d9d9")
        self.Button_step_x.configure(text='''x''')

        self.Label_step = Label(self.MM_Frame)
        self.Label_step.place(relx=0.049, rely=0.441, height=18, width=66)
        self.Label_step.configure(activebackground="#f9f9f9")
        self.Label_step.configure(text='''Step (um)''')

        self.Button_step_y = Button(self.MM_Frame, command=lambda: step_y())
        self.Button_step_y.place(relx=0.198, rely=0.61, height=26, width=32)
        self.Button_step_y.configure(activebackground="#d9d9d9")
        self.Button_step_y.configure(text='''y''')

        self.Button_step_z = Button(self.MM_Frame, command=lambda: step_z())
        self.Button_step_z.place(relx=0.198, rely=0.712, height=26, width=32)
        self.Button_step_z.configure(activebackground="#d9d9d9")
        self.Button_step_z.configure(text='''z''')

        self.Button_setorigin = Button(self.MM_Frame)
        self.Button_setorigin.place(relx=0.049, rely=0.305, height=26, width=87)
        self.Button_setorigin.configure(activebackground="#d9d9d9")
        self.Button_setorigin.configure(text='''Set Origin''')

        self.Button_velocity = Button(self.MM_Frame)
        self.Button_velocity.place(relx=0.395, rely=0.441, height=26, width=170)
        self.Button_velocity.configure(activebackground="#d9d9d9")
        self.Button_velocity.configure(text='''Change Velocity (um/s)''')

        self.Entry_velocity = Entry(self.MM_Frame)
        self.Entry_velocity.place(relx=0.815, rely=0.441, height=20
                                  , relwidth=0.138)
        self.Entry_velocity.configure(background="white")
        self.Entry_velocity.configure(font="TkFixedFont")
        self.Entry_velocity.configure(selectbackground="#c4c4c4")
        self.Entry_velocity.configure(textvariable=gui_support.velocity)

        self.Spinbox_step_x = Spinbox(self.MM_Frame, from_=-50.0, to=50.0)
        self.Spinbox_step_x.place(relx=0.099, rely=0.508, relheight=0.068
                                  , relwidth=0.094)
        self.Spinbox_step_x.configure(activebackground="#f9f9f9")
        self.Spinbox_step_x.configure(background="white")
        self.Spinbox_step_x.configure(from_="-50.0")
        self.Spinbox_step_x.configure(highlightbackground="black")
        self.Spinbox_step_x.configure(selectbackground="#c4c4c4")
        self.Spinbox_step_x.configure(textvariable=gui_support.step_x)
        self.Spinbox_step_x.configure(to="50.0")

        self.Spinbox_step_z = Spinbox(self.MM_Frame, from_=-50.0, to=50.0)
        self.Spinbox_step_z.place(relx=0.099, rely=0.712, relheight=0.068
                                  , relwidth=0.094)
        self.Spinbox_step_z.configure(activebackground="#f9f9f9")
        self.Spinbox_step_z.configure(background="white")
        self.Spinbox_step_z.configure(from_="-50.0")
        self.Spinbox_step_z.configure(highlightbackground="black")
        self.Spinbox_step_z.configure(selectbackground="#c4c4c4")
        self.Spinbox_step_z.configure(textvariable=gui_support.step_z)
        self.Spinbox_step_z.configure(to="50.0")

        self.Spinbox_step_y = Spinbox(self.MM_Frame, from_=-50.0, to=50.0)
        self.Spinbox_step_y.place(relx=0.099, rely=0.61, relheight=0.068
                                  , relwidth=0.094)
        self.Spinbox_step_y.configure(activebackground="#f9f9f9")
        self.Spinbox_step_y.configure(background="white")
        self.Spinbox_step_y.configure(from_="-50.0")
        self.Spinbox_step_y.configure(highlightbackground="black")
        self.Spinbox_step_y.configure(selectbackground="#c4c4c4")
        self.Spinbox_step_y.configure(textvariable=gui_support.step_y)
        self.Spinbox_step_y.configure(to="50.0")

        self.Radiobutton_highres = Radiobutton(self.MM_Frame)
        self.Radiobutton_highres.place(relx=0.42, rely=0.576, relheight=0.068
                                       , relwidth=0.311)
        self.Radiobutton_highres.configure(activebackground="#d9d9d9")
        self.Radiobutton_highres.configure(justify=LEFT)
        self.Radiobutton_highres.configure(text='''High Resolution''')
        self.Radiobutton_highres.configure(value="high")
        self.Radiobutton_highres.configure(variable=gui_support.radio_resolution)

        self.Radiobutton_lowres = Radiobutton(self.MM_Frame)
        self.Radiobutton_lowres.place(relx=0.42, rely=0.644, relheight=0.068
                                      , relwidth=0.299)
        self.Radiobutton_lowres.configure(activebackground="#d9d9d9")
        self.Radiobutton_lowres.configure(justify=LEFT)
        self.Radiobutton_lowres.configure(text='''Low Resolution''')
        self.Radiobutton_lowres.configure(value="low")
        self.Radiobutton_lowres.configure(variable=gui_support.radio_resolution)

        self.Button_mm_interrupt = Button(self.MM_Frame)
        self.Button_mm_interrupt.place(relx=0.765, rely=0.881, height=26
                                       , width=81)
        self.Button_mm_interrupt.configure(activebackground="#d9d9d9")
        self.Button_mm_interrupt.configure(state=ACTIVE)
        self.Button_mm_interrupt.configure(text='''Interrupt''')

        self.Label_pathing = Label(self.MM_Frame)
        self.Label_pathing.place(relx=0.519, rely=0.136, height=18, width=110)
        self.Label_pathing.configure(activebackground="#f9f9f9")
        self.Label_pathing.configure(text='''Pathing Function''')

        self.Entry_pathing_func = Entry(self.MM_Frame)
        self.Entry_pathing_func.place(relx=0.568, rely=0.203, height=20
                                      , relwidth=0.336)
        self.Entry_pathing_func.configure(background="white")
        self.Entry_pathing_func.configure(font="TkFixedFont")
        self.Entry_pathing_func.configure(selectbackground="#c4c4c4")
        self.Entry_pathing_func.configure(textvariable=gui_support.pathing_func)

        self.Current_Frame = Frame(top)
        self.Current_Frame.place(relx=0.0, rely=0.477, relheight=0.517
                                 , relwidth=0.534)
        self.Current_Frame.configure(relief=RAISED)
        self.Current_Frame.configure(borderwidth="2")
        self.Current_Frame.configure(relief=RAISED)
        self.Current_Frame.configure(width=405)

        self.Label_ps = Label(self.Current_Frame)
        self.Label_ps.place(relx=0.025, rely=0.031, height=23, width=175)
        self.Label_ps.configure(activebackground="#f9f9f9")
        self.Label_ps.configure(font=font9)
        self.Label_ps.configure(text='''Current/Power Supply''')

        self.Button_ps_interrupt = Button(self.Current_Frame)
        self.Button_ps_interrupt.place(relx=0.765, rely=0.862, height=26
                                       , width=81)
        self.Button_ps_interrupt.configure(activebackground="#d9d9d9")
        self.Button_ps_interrupt.configure(text='''Interrupt''')

        self.style.configure('TNotebook.Tab', background=_bgcolor)
        self.style.configure('TNotebook.Tab', foreground=_fgcolor)
        self.style.map('TNotebook.Tab', background=
        [('selected', _compcolor), ('active', _ana2color)])
        self.Notebook_ps = ttk.Notebook(self.Current_Frame)
        self.Notebook_ps.place(relx=0.025, rely=0.123, relheight=0.686
                               , relwidth=0.919)
        self.Notebook_ps.configure(width=372)
        self.Notebook_ps.configure(takefocus="")
        self.Notebook_ps_t0 = Frame(self.Notebook_ps)
        self.Notebook_ps.add(self.Notebook_ps_t0, padding=3)
        self.Notebook_ps.tab(0, text="Constant", compound="left", underline="-1"
                             , )
        self.Notebook_ps_t1 = Frame(self.Notebook_ps)
        self.Notebook_ps.add(self.Notebook_ps_t1, padding=3)
        self.Notebook_ps.tab(1, text="Square", compound="left", underline="-1", )
        self.Notebook_ps_t2 = Frame(self.Notebook_ps)
        self.Notebook_ps.add(self.Notebook_ps_t2, padding=3)
        self.Notebook_ps.tab(2, text="Custom", compound="left", underline="-1", )
        self.Notebook_ps_t3 = Frame(self.Notebook_ps)
        self.Notebook_ps.add(self.Notebook_ps_t3, padding=3)
        self.Notebook_ps.tab(3, text="Sinusoidal", compound="left", underline="-1"
                             , )

        self.Label_constant_amps = Label(self.Notebook_ps_t0)
        self.Label_constant_amps.place(relx=0.027, rely=0.05, height=18
                                       , width=92)
        self.Label_constant_amps.configure(activebackground="#f9f9f9")
        self.Label_constant_amps.configure(text='''Amperes (mA)''')

        self.Entry_constant_amps = Entry(self.Notebook_ps_t0)
        self.Entry_constant_amps.place(relx=0.297, rely=0.05, height=20
                                       , relwidth=0.124)
        self.Entry_constant_amps.configure(background="white")
        self.Entry_constant_amps.configure(font="TkFixedFont")
        self.Entry_constant_amps.configure(selectbackground="#c4c4c4")
        self.Entry_constant_amps.configure(textvariable=gui_support.constant_amps)

        self.Label_constant_duration = Label(self.Notebook_ps_t0)
        self.Label_constant_duration.place(relx=0.027, rely=0.15, height=18
                                           , width=79)
        self.Label_constant_duration.configure(activebackground="#f9f9f9")
        self.Label_constant_duration.configure(text='''Duration (s)''')

        self.Entry_constant_duration = Entry(self.Notebook_ps_t0)
        self.Entry_constant_duration.place(relx=0.297, rely=0.15, height=20
                                           , relwidth=0.124)
        self.Entry_constant_duration.configure(background="white")
        self.Entry_constant_duration.configure(font="TkFixedFont")
        self.Entry_constant_duration.configure(selectbackground="#c4c4c4")
        self.Entry_constant_duration.configure(textvariable=gui_support.constant_duration)

        self.Button_constant_setconfig = Button(self.Notebook_ps_t0)
        self.Button_constant_setconfig.place(relx=0.027, rely=0.3, height=26
                                             , width=135)
        self.Button_constant_setconfig.configure(activebackground="#d9d9d9")
        self.Button_constant_setconfig.configure(text='''Set Configuration''')

        self.Label8 = Label(self.Notebook_ps_t0)
        self.Label8.place(relx=0.486, rely=0.15, height=18, width=42)
        self.Label8.configure(activebackground="#f9f9f9")
        self.Label8.configure(text='''Units?''')

        self.Label_square_amps = Label(self.Notebook_ps_t1)
        self.Label_square_amps.place(relx=0.027, rely=0.05, height=18, width=92)
        self.Label_square_amps.configure(activebackground="#f9f9f9")
        self.Label_square_amps.configure(text='''Amperes (mA)''')

        self.Entry_square_amps = Entry(self.Notebook_ps_t1)
        self.Entry_square_amps.place(relx=0.297, rely=0.05, height=20
                                     , relwidth=0.124)
        self.Entry_square_amps.configure(background="white")
        self.Entry_square_amps.configure(font="TkFixedFont")
        self.Entry_square_amps.configure(selectbackground="#c4c4c4")
        self.Entry_square_amps.configure(textvariable=gui_support.square_amps)

        self.Label_square_duration = Label(self.Notebook_ps_t1)
        self.Label_square_duration.place(relx=0.027, rely=0.15, height=18
                                         , width=79)
        self.Label_square_duration.configure(activebackground="#f9f9f9")
        self.Label_square_duration.configure(text='''Duration (s)''')

        self.Entry_square_duration = Entry(self.Notebook_ps_t1)
        self.Entry_square_duration.place(relx=0.297, rely=0.15, height=20
                                         , relwidth=0.124)
        self.Entry_square_duration.configure(background="white")
        self.Entry_square_duration.configure(font="TkFixedFont")
        self.Entry_square_duration.configure(selectbackground="#c4c4c4")
        self.Entry_square_duration.configure(textvariable=gui_support.square_duration)

        self.Label_square_duty = Label(self.Notebook_ps_t1)
        self.Label_square_duty.place(relx=0.027, rely=0.25, height=18, width=93)
        self.Label_square_duty.configure(activebackground="#f9f9f9")
        self.Label_square_duty.configure(text='''Duty Cycle (%)''')

        self.Entry_square_duty = Entry(self.Notebook_ps_t1)
        self.Entry_square_duty.place(relx=0.297, rely=0.25, height=20
                                     , relwidth=0.124)
        self.Entry_square_duty.configure(background="white")
        self.Entry_square_duty.configure(font="TkFixedFont")
        self.Entry_square_duty.configure(selectbackground="#c4c4c4")
        self.Entry_square_duty.configure(textvariable=gui_support.square_duty)

        self.Button_square_setconfig = Button(self.Notebook_ps_t1)
        self.Button_square_setconfig.place(relx=0.027, rely=0.5, height=26
                                           , width=135)
        self.Button_square_setconfig.configure(activebackground="#d9d9d9")
        self.Button_square_setconfig.configure(text='''Set Configuration''')

        self.Label_square_freq = Label(self.Notebook_ps_t1)
        self.Label_square_freq.place(relx=0.027, rely=0.35, height=18, width=97)
        self.Label_square_freq.configure(activebackground="#f9f9f9")
        self.Label_square_freq.configure(text='''Frequency (Hz)''')

        self.Entry_square_freq = Entry(self.Notebook_ps_t1)
        self.Entry_square_freq.place(relx=0.297, rely=0.35, height=20
                                     , relwidth=0.124)
        self.Entry_square_freq.configure(background="white")
        self.Entry_square_freq.configure(font="TkFixedFont")
        self.Entry_square_freq.configure(selectbackground="#c4c4c4")
        self.Entry_square_freq.configure(textvariable=gui_support.square_freq)

        self.Label9 = Label(self.Notebook_ps_t2)
        self.Label9.place(relx=0.054, rely=0.15, height=98, width=217)
        self.Label9.configure(activebackground="#f9f9f9")
        self.Label9.configure(justify=LEFT)
        self.Label9.configure(text='''linear rise, exponential rise, rise time, duration-risetime=steadystate time''')
        self.Label9.configure(wraplength="200")

        self.Entry_sin_freq = Entry(self.Notebook_ps_t3)
        self.Entry_sin_freq.place(relx=0.324, rely=0.35, height=20
                                  , relwidth=0.124)
        self.Entry_sin_freq.configure(background="white")
        self.Entry_sin_freq.configure(font="TkFixedFont")
        self.Entry_sin_freq.configure(selectbackground="#c4c4c4")
        self.Entry_sin_freq.configure(textvariable=gui_support.sin_freq)

        self.Label_sin_freq = Label(self.Notebook_ps_t3)
        self.Label_sin_freq.place(relx=0.027, rely=0.35, height=18, width=97)
        self.Label_sin_freq.configure(activebackground="#f9f9f9")
        self.Label_sin_freq.configure(text='''Frequency (Hz)''')

        self.Label_sin_amplitude = Label(self.Notebook_ps_t3)
        self.Label_sin_amplitude.place(relx=0.027, rely=0.05, height=18
                                       , width=99)
        self.Label_sin_amplitude.configure(activebackground="#f9f9f9")
        self.Label_sin_amplitude.configure(text='''Amplitude (mA)''')

        self.Entry_sin_amplitude = Entry(self.Notebook_ps_t3)
        self.Entry_sin_amplitude.place(relx=0.324, rely=0.05, height=20
                                       , relwidth=0.124)
        self.Entry_sin_amplitude.configure(background="white")
        self.Entry_sin_amplitude.configure(font="TkFixedFont")
        self.Entry_sin_amplitude.configure(selectbackground="#c4c4c4")
        self.Entry_sin_amplitude.configure(textvariable=gui_support.sin_amplitude)

        self.Label_sin_offset = Label(self.Notebook_ps_t3)
        self.Label_sin_offset.place(relx=0.027, rely=0.15, height=18, width=74)
        self.Label_sin_offset.configure(activebackground="#f9f9f9")
        self.Label_sin_offset.configure(text='''Offset (mA)''')

        self.Entry_sin_offset = Entry(self.Notebook_ps_t3)
        self.Entry_sin_offset.place(relx=0.324, rely=0.15, height=20
                                    , relwidth=0.124)
        self.Entry_sin_offset.configure(background="white")
        self.Entry_sin_offset.configure(font="TkFixedFont")
        self.Entry_sin_offset.configure(selectbackground="#c4c4c4")
        self.Entry_sin_offset.configure(textvariable=gui_support.sin_offset)

        self.Label_sin_duration = Label(self.Notebook_ps_t3)
        self.Label_sin_duration.place(relx=0.027, rely=0.25, height=18, width=79)

        self.Label_sin_duration.configure(activebackground="#f9f9f9")
        self.Label_sin_duration.configure(text='''Duration (s)''')

        self.Entry_sin_duration = Entry(self.Notebook_ps_t3)
        self.Entry_sin_duration.place(relx=0.324, rely=0.25, height=20
                                      , relwidth=0.124)
        self.Entry_sin_duration.configure(background="white")
        self.Entry_sin_duration.configure(font="TkFixedFont")
        self.Entry_sin_duration.configure(selectbackground="#c4c4c4")
        self.Entry_sin_duration.configure(textvariable=gui_support.sin_duration)

        self.Button_sin_setconfig = Button(self.Notebook_ps_t3)
        self.Button_sin_setconfig.place(relx=0.027, rely=0.5, height=26
                                        , width=135)
        self.Button_sin_setconfig.configure(activebackground="#d9d9d9")
        self.Button_sin_setconfig.configure(text='''Set Configuration''')

        self.Button_ps_run = Button(self.Current_Frame)
        self.Button_ps_run.place(relx=0.049, rely=0.862, height=26, width=50)
        self.Button_ps_run.configure(activebackground="#d9d9d9")
        self.Button_ps_run.configure(text='''Run''')

        self.Button_ps_stop = Button(self.Current_Frame)
        self.Button_ps_stop.place(relx=0.272, rely=0.862, height=26, width=55)
        self.Button_ps_stop.configure(activebackground="#d9d9d9")
        self.Button_ps_stop.configure(text='''Stop''')

        self.Status_Frame = Frame(top)
        self.Status_Frame.place(relx=0.541, rely=0.0, relheight=0.676
                                , relwidth=0.455)
        self.Status_Frame.configure(relief=RAISED)
        self.Status_Frame.configure(borderwidth="2")
        self.Status_Frame.configure(relief=RAISED)
        self.Status_Frame.configure(width=345)

        self.Label_status = Label(self.Status_Frame)
        self.Label_status.place(relx=0.029, rely=0.024, height=23, width=54)
        self.Label_status.configure(activebackground="#f9f9f9")
        self.Label_status.configure(font=font9)
        self.Label_status.configure(text='''Status''')

        self.Label_status_pos = Label(self.Status_Frame)
        self.Label_status_pos.place(relx=0.058, rely=0.094, height=18, width=91)
        self.Label_status_pos.configure(activebackground="#f9f9f9")
        self.Label_status_pos.configure(text='''Position (um):''')

        self.Label_status_res = Label(self.Status_Frame)
        self.Label_status_res.place(relx=0.058, rely=0.188, height=18, width=74)
        self.Label_status_res.configure(activebackground="#f9f9f9")
        self.Label_status_res.configure(text='''Resolution:''')

        self.Label_status_vel = Label(self.Status_Frame)
        self.Label_status_vel.place(relx=0.058, rely=0.141, height=18, width=107)

        self.Label_status_vel.configure(activebackground="#f9f9f9")
        self.Label_status_vel.configure(text='''Velocity (um/s):''')

        self.Label_status_magfield = Label(self.Status_Frame)
        self.Label_status_magfield.place(relx=0.058, rely=0.235, height=18
                                         , width=98)
        self.Label_status_magfield.configure(activebackground="#f9f9f9")
        self.Label_status_magfield.configure(text='''Magnetic Field:''')

        self.Label_console = Label(self.Status_Frame)
        self.Label_console.place(relx=0.058, rely=0.535, height=18, width=101)
        self.Label_console.configure(activebackground="#f9f9f9")
        self.Label_console.configure(text='''Console Output''')

        self.Label_status_pos_v = Label(self.Status_Frame)
        self.Label_status_pos_v.place(relx=0.348, rely=0.094, height=18
                                      , width=39)
        self.Label_status_pos_v.configure(activebackground="#f9f9f9")
        self.Label_status_pos_v.configure(justify=LEFT)
        self.Label_status_pos_v.configure(text='''Value''')
        self.Label_status_pos_v.configure(textvariable=gui_support.status_pos_v)

        self.Label_status_vel_v = Label(self.Status_Frame)
        self.Label_status_vel_v.place(relx=0.377, rely=0.141, height=18
                                      , width=39)
        self.Label_status_vel_v.configure(activebackground="#f9f9f9")
        self.Label_status_vel_v.configure(justify=LEFT)
        self.Label_status_vel_v.configure(text='''Value''')
        self.Label_status_vel_v.configure(textvariable=gui_support.status_vel_v)

        self.Label_status_res_v = Label(self.Status_Frame)
        self.Label_status_res_v.place(relx=0.29, rely=0.188, height=18, width=39)

        self.Label_status_res_v.configure(activebackground="#f9f9f9")
        self.Label_status_res_v.configure(justify=LEFT)
        self.Label_status_res_v.configure(text='''Value''')
        self.Label_status_res_v.configure(textvariable=gui_support.status_res_v)

        self.Label_status_magfield_v = Label(self.Status_Frame)
        self.Label_status_magfield_v.place(relx=0.377, rely=0.235, height=18
                                           , width=39)
        self.Label_status_magfield_v.configure(activebackground="#f9f9f9")
        self.Label_status_magfield_v.configure(justify=LEFT)
        self.Label_status_magfield_v.configure(text='''Value''')
        self.Label_status_magfield_v.configure(textvariable=gui_support.status_magfield_v)

        self.Button_status_refresh = Button(self.Status_Frame)
        self.Button_status_refresh.place(relx=0.754, rely=0.024, height=26
                                         , width=74)
        self.Button_status_refresh.configure(activebackground="#d9d9d9")
        self.Button_status_refresh.configure(text='''Refresh''')

        self.Button_master_stop = Button(self.Status_Frame)
        self.Button_master_stop.place(relx=0.348, rely=0.918, height=31
                                      , width=102)
        self.Button_master_stop.configure(activebackground="#d9d9d9")
        self.Button_master_stop.configure(font=font9)
        self.Button_master_stop.configure(foreground="#ff0000")
        self.Button_master_stop.configure(text='''STOP ALL''')

        self.console_output = ScrolledText(self.Status_Frame)
        self.console_output.place(relx=0.029, rely=0.588, relheight=0.325
                                  , relwidth=0.951)
        self.console_output.configure(background="white")
        self.console_output.configure(font="TkTextFont")
        self.console_output.configure(insertborderwidth="3")
        self.console_output.configure(selectbackground="#c4c4c4")
        self.console_output.configure(width=10)
        self.console_output.configure(wrap=NONE)

        self.Frame_demag = Frame(top)
        self.Frame_demag.place(relx=0.541, rely=0.684, relheight=0.31
                               , relwidth=0.455)
        self.Frame_demag.configure(relief=RAISED)
        self.Frame_demag.configure(borderwidth="2")
        self.Frame_demag.configure(relief=RAISED)
        self.Frame_demag.configure(width=345)

        self.Button_demag = Button(self.Frame_demag, command=lambda:demagnetization())
        self.Button_demag.place(relx=0.145, rely=0.256, height=58, width=240)
        self.Button_demag.configure(activebackground="#d9d9d9")
        self.Button_demag.configure(text='''Start Demagnetization Process''')

        self.Label1 = Label(self.Frame_demag)
        self.Label1.place(relx=0.145, rely=0.615, height=18, width=193)
        self.Label1.configure(activebackground="#f9f9f9")
        self.Label1.configure(text='''tkinter messagebox for popup''')


class Controller():
    def __init__(self,gui_instance, man_instance):
        self.gui_instance=gui_instance
        self.man_instance=man_instance

    def status_refresh(self):
        curPos=self.man_instance.get_current_position(self.man_instance)
        gui_support.status_pos=curPos   #Update Position
        gui_support.status_res_v=gui_support.velocity   #Update velocity - Can set status label to entry label (Instant Update)?



# The following code is added to facilitate the Scrolled widgets you specified.
class AutoScroll(object):
    '''Configure the scrollbars for a widget.'''

    def __init__(self, master):
        #  Rozen. Added the try-except clauses so that this class
        #  could be used for scrolled entry widget for which vertical
        #  scrolling is not supported. 5/7/14.
        try:
            vsb = ttk.Scrollbar(master, orient='vertical', command=self.yview)
        except:
            pass
        hsb = ttk.Scrollbar(master, orient='horizontal', command=self.xview)

        #self.configure(yscrollcommand=_autoscroll(vsb),
        #    xscrollcommand=_autoscroll(hsb))
        try:
            self.configure(yscrollcommand=self._autoscroll(vsb))
        except:
            pass
        self.configure(xscrollcommand=self._autoscroll(hsb))

        self.grid(column=0, row=0, sticky='nsew')
        try:
            vsb.grid(column=1, row=0, sticky='ns')
        except:
            pass
        hsb.grid(column=0, row=1, sticky='ew')

        master.grid_columnconfigure(0, weight=1)
        master.grid_rowconfigure(0, weight=1)

        # Copy geometry methods of master  (taken from ScrolledText.py)
        if py3:
            methods = Pack.__dict__.keys() | Grid.__dict__.keys() \
                  | Place.__dict__.keys()
        else:
            methods = Pack.__dict__.keys() + Grid.__dict__.keys() \
                  + Place.__dict__.keys()

        for meth in methods:
            if meth[0] != '_' and meth not in ('config', 'configure'):
                setattr(self, meth, getattr(master, meth))

    @staticmethod
    def _autoscroll(sbar):
        '''Hide and show scrollbar as needed.'''
        def wrapped(first, last):
            first, last = float(first), float(last)
            if first <= 0 and last >= 1:
                sbar.grid_remove()
            else:
                sbar.grid()
            sbar.set(first, last)
        return wrapped

    def __str__(self):
        return str(self.master)

def _create_container(func):
    '''Creates a ttk Frame with a given master, and use this new frame to
    place the scrollbars and the widget.'''
    def wrapped(cls, master, **kw):
        container = ttk.Frame(master)
        return func(cls, container, **kw)
    return wrapped

class ScrolledText(AutoScroll, Text):
    '''A standard Tkinter Text widget with scrollbars that will
    automatically show/hide as needed.'''
    @_create_container
    def __init__(self, master, **kw):
        Text.__init__(self, master, **kw)
        AutoScroll.__init__(self, master)

if __name__ == '__main__':
    vp_start_gui()



