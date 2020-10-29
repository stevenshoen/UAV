#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 19 17:49:10 2020

@author: pi
"""
from kivy.app import App
from kivy.uix.button import Button

class MyApp(App):
    def build(self):
        return Button(text='',
                      background_color=(0, 0, 1, 1))

if __name__ == '__main__':
    MyApp().run()
