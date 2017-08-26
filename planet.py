import numpy as np
import random

G = .1

class WeightingObject:
    def __init__(self,x,y,z,mass):
        self.x = [x,y,z]
        self.mass = mass
        self.density = 50000.
        self.radius = (0.75 / np.pi * (self.mass/self.density))**(1./3.)
        self.force = [0., 0., 0.]
        self.speed = [5.*(0.5-random.random()), 5.*(0.5-random.random()), 5.*(0.5-random.random())]

    def acc(x):
        r2 = ((self.x[0] - x[0])**2.,(self.x[1] - x[1])**2.,(self.x[2] - x[2])**2.)
        if r2>1.e-3 : 
            return G*self.mass/r2
        else:
            return 0.


    def __str__(self):
        s = ''
        s+=str( (str(self.xdd),str(self.mass)))
        return s

class Cosmos(list):
    def __init__(self,n=5):
        super(Cosmos, self).__init__()
        for i in range(n):
            x = (0.5-random.random())*10.
            y = (0.5-random.random())*10.
            z = (0.5-random.random())*10.
            mass = random.random()*50.
            self.append(WeightingObject(x,y,z,mass))
        n=n+1
        for i in range(n):
            self.matr = np.zeros((n,n),dtype=np.float)
            self.matr2 = np.zeros((n,n),dtype=np.float)

    def output_array(self):
        out = []
        for o in self :
            out.append([o.x[0], o.x[1], o.x[2], 0., 1., 0., o.radius])
        return np.array(out)


    def update_mat(self,dt,sun = False):
        if sun == True:
            print "zob"
        for o in self : o.force = [0., 0., 0. ]
        for i in range(len(self)) :
            for j in range(i,len(self)) :
                if (j!=i) :
                    oi = self[i]
                    oj = self[j]
                    if ((oi.x[0] - oj.x[0])**2. + (oi.x[1] - oj.x[1])**2. + (oi.x[2] - oj.x[2])**2.) <= (oi.radius+oj.radius) : 
                        self[j].mass = oi.mass+oj.mass
                        self[j].radius = (0.75 / np.pi * (self[j].mass/self[j].density))**(1./3.)
                        self[j].speed = [ (oi.mass*oi.speed[k] + oj.mass*oj.speed[k]) / (oi.mass+oj.mass) for  k in range(3)]
                        self[j].x = [ (oi.mass*oi.x[k] + oj.mass*oj.x[k]) / (oi.mass+oj.mass) for  k in range(3)]
                        self.remove(oi)
                        break
        for i in range(len(self)) :
            for j in range(i,len(self)) :
                if (j!=i) :
                    oi = self[i]
                    oj = self[j]
                    # planet i applies a force on planet j that is oriented by j -> i  = uji
                    uji = [ (oi.x[0] - oj.x[0]) , (oi.x[1] - oj.x[1]) , (oi.x[2] - oj.x[2]) ]
                    uij = [ -uji[k] for k in range(3) ]
                    self.matr[i,j] = np.sqrt((oi.x[0] - oj.x[0])**2. + (oi.x[1] - oj.x[1])**2. + (oi.x[2] - oj.x[2])**2.)
                    self.matr[j,i] =  self.matr[i,j] 
                    oi.force = [ oi.force[k] + G*oi.mass*oj.mass*uij[k]/self.matr[i,j]**3. for k in range(3)] 
                    oj.force = [ oj.force[k] - G*oi.mass*oj.mass*uij[k]/self.matr[i,j]**3. for k in range(3)]

        
        out = []
        for o in self :
            o.speed = [o.speed[i] + dt * o.force[i]/o.mass for i in range(3)]
            if np.sum([u**2. for u in o.x])>2000.:o.speed = [ -0.9*v for v in o.speed]
            o.x = [o.x[i] + dt * o.speed[i] for i in range(3)]
            out.append([o.x[0], o.x[1], o.x[2], 0., 1., 0., o.radius])
        return np.array(out)
         

if __name__ == '__main__':
    co = Cosmos(n=2000)
    for t in range(10000):
        print '---------------------------------'
        co.update_mat(0.1)
