from PyQt5 import QtWidgets, QtOpenGL
from OpenGL.GL import *
import sys

app = QtWidgets.QApplication(sys.argv)
widget = QtOpenGL.QGLWidget()
widget.show()
print("GL context created successfully.")
sys.exit(app.exec_())
