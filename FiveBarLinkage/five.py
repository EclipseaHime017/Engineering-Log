import numpy as np
import matplotlib.pyplot as plt

class five:
    def __init__(self, l1 = 0.25, l2 = 0.25, l3 = 0.25, l4 = 0.25, l5 = 0.2):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.l5 = l5
        self.X_C = 0.0
        self.Y_C = 0.4
        
    def update_position(self,X_C,Y_C):
        self.X_C = X_C
        self.Y_C = Y_C
        
    def inverse_kinematics(self):
        a = (self.X_C + self.l5/2)**2 + self.Y_C**2
        b = (self.X_C - self.l5/2)**2 + self.Y_C**2
        k1 = self.l1 **2  + self.l2 **2
        k2 = 2 * self.l1 * self.l2
        k3 = self.l3 **2  + self.l4 **2
        k4 = 2 * self.l3 * self.l4
        
        phi2 = -np.arccos((a - k1) / k2)
        phi3 = np.arccos((b - k3) / k4)
        
        # c = self.Y_C / (self.X_C + self.l5/2) * (self.l1 + self.l2 * np.cos(phi2)) - self.l2 * np.sin(phi2)
        # d = self.l1 + self.l2 * np.cos(phi2) + self.Y_C / (self.X_C + self.l5/2) * self.l2 * np.sin(phi2)
        # phi1 = np.arctan2(c,d)
        
        # e = self.Y_C / (self.X_C - self.l5/2) * (self.l3 + self.l4 * np.cos(phi3)) - self.l4 * np.sin(phi3)
        # f = self.l3 + self.l4 * np.cos(phi3) + self.Y_C / (self.X_C - self.l5/2) * self.l4 * np.sin(phi3)
        # phi4 = np.arctan2(e,f)
        
        c = (-(self.X_C + self.l5/2) * self.l2 * np.sin(phi2) + self.Y_C * (self.l1 + self.l2 * np.cos(phi2)))
        d = ((self.l1 + self.l2 * np.cos(phi2)) * (self.X_C + self.l5/2) + self.Y_C * self.l2 * np.sin(phi2))
        phi1 = np.arctan2(c,d)
        
        e = (-(self.X_C - self.l5/2) * self.l3 * np.sin(phi3) + self.Y_C * (self.l4 + self.l3 * np.cos(phi3)))
        f = ((self.l4 + self.l3 * np.cos(phi3)) * (self.X_C - self.l5/2) + self.Y_C * self.l3 * np.sin(phi3))
        phi4 = np.arctan2(e,f)
        return phi1, phi2, phi3, phi4
    
def main():
    link = five(l5 = 0.2)
    link.update_position(0.1,0.38)
    phi1, phi2, phi3, phi4 = link.inverse_kinematics()
    print("phi1:", phi1 * 180 / np.pi)
    print("phi2:", phi2 * 180 / np.pi)
    print("phi3:", phi3 * 180 / np.pi)
    print("phi4:", phi4 * 180 / np.pi)
    
if __name__ == '__main__':
    main()