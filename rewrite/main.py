import threading
import cv2
import gi
import time
import math
import os.path
import sys
import tkinter as tk
from PIL import ImageTk, Image
from pymavlink.quaternion import QuaternionBase
from pymavlink import mavutil
import numpy as np
gi.require_version("Gst", "1.0")
from gi.repository import Gst
from .videoclass import Video

video = Video(port=4777)

