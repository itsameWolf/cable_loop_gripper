import numpy as np

class VialTray:
    def __init__(self, rows=3, cols=3, vial_r=0.012, vial_spacing=0.026, tray_origin=(0.2,0.2,0.0), z_drop=0.05):
        self._rows = rows
        self._cols = cols
        self._n_vials = self._rows*self._cols

        self._vial_r = vial_r
        self._vial_spacing = vial_spacing
        self._tray_origin = np.array(tray_origin)

        self._z_drop = z_drop

        self._vial_positions = []
        for i in range(self._rows):
            for j in range(self._cols):
                delta = np.array([i*self._vial_spacing, -j*self._vial_spacing, 0.0])
                self._vial_positions.append(self._tray_origin+delta)
          
    def vial_positions(self, i=None):
        if i==None:
            return self._vial_positions
        else:
            return self._vial_positions[i]

    def n_vials(self):
        return self._n_vials