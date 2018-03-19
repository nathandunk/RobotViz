import pyqtgraph as pg
import sys
import pyqtgraph.opengl as gl
from numpy import *

if __name__ == "__main__":

    pg.mkQApp()

    ## make a widget for displaying 3D objects
    import pyqtgraph.opengl as gl
    view = gl.GLViewWidget()
    view.show()

    ## create three grids, add each to the view
    xgrid = gl.GLGridItem()
    ygrid = gl.GLGridItem()
    zgrid = gl.GLGridItem()
    view.addItem(xgrid)
    view.addItem(ygrid)
    view.addItem(zgrid)

    # ## rotate x and y grids to face the correct direction
    xgrid.rotate(90, 0, 1, 0)
    ygrid.rotate(90, 1, 0, 0)

    ## scale each grid differently
    xgrid.scale(1, 1, 1)
    ygrid.scale(1, 1, 1)
    zgrid.scale(1, 1, 1)

    X = linspace(-10,10,100)
    Y1=2+sin(X)
    Y2=-2+Y1*Y1
    Z = linspace(-10,10,100)
    p=array([X,Y2,Z])
    # X = 

    p=p.transpose() 

    C=pg.glColor('w')

    plt = gl.GLLinePlotItem(pos=p, width=2, color=C)


    view.addItem(plt)  ## setting pen=None disables line drawing

    ## Switch to using white background and black foreground
    # pg.setConfigOption('background', 'w')
    # pg.setConfigOption('foreground', 'k')

    if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):
        pg.QtGui.QApplication.exec_()