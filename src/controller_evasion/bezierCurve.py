from math import *
NT = float

from matplotlib.pylab import *

# class Point
class Point:
    def __init__(self, x = 0.0, y = 0.0):
        self.x = x
        self.y = y

    def distance(self, p):
        return sqrt((p.x-self.x)*(p.x-self.x)+(p.y-self.y)*(p.y-self.y))

    def length(self):
        return self.distance(Point(NT(0), NT(0)))

    def __sub__(self, p):
        return Point(self.x-p.x, self.y-p.y)

    def __add__(self, p):
        return Point(self.x+p.x, self.y+p.y)

    def __mul__(self, c):
        return Point(c*self.x, c*self.y)

    def __eq__(self, p):
        return self.x == p.x and self.y == p.y

    def __ne__(self, p):
        return not (self == p)

    def towards(self, target, t):
        if t == 0.5:
            return self.halfway(target)
        else:
            return Point((1.0-t)*self.x+t*target.x, (1.0-t)*self.y+t*target.y)

    def halfway(self, target):
        return Point((self.x+target.x).div2(), (self.y+target.y).div2())

    def compare_lex(self, p):
        if self.x < p.x:
            return -1
        if self.x > p.x:
            return 1
        if self.y < p.y:
            return -1
        if self.y > p.y:
            return 1
        return 0

    def less_lex(self, p):
        return self.compare_lex(p) < 0

    def less_eq_lex(self, p):
        return self.compare_lex(p) <= 0

    def __repr__(self):
        return "Point(%s, %s)" % (self.x, self.y)

def orientation_2d(a, b, c):
    d1 = (a.x - b.x) * (a.y - c.y)
    d2 = (a.y - b.y) * (a.x - c.x)
    if d1 == d2:
        return 0
    elif d1 > d2:
        return 1
    else:
        return -1

def left_turn(a, b, c):
    return orientation_2d(a, b, c) > 0

def right_turn(a, b, c):
    return orientation_2d(a, b, c) < 0

def between_var(a, b, c):
    return (a.x-b.x)*(c.x-b.x)+(a.y-b.y)*(c.y-b.y)<0

# class CHull
class CHull:
    def __init__(self, vp):
        self.n = len(vp)
        self.vp = vp
        self.isConvex = False
        self.diam = 0



# class Bezier
class Bezier(CHull):

   def __init__(self, v):
        self.deg = len(v) - 1
        self.cp = v
        self.tmin = NT(0)
        self.tmax = NT(1)


   def get_point(self, t):
        curr = [0]*self.deg
        # get initial
        for i in range(self.deg):
            curr[i] = self.cp[i].towards(self.cp[i+1], t)
        for i in range(self.deg-1):
            for j in range(self.deg-1-i):
                curr[j] = curr[j].towards(curr[j+1], t)
        return curr[0]

   def subdivision(self, t):
        lseq = [0]*(self.deg+1)
        rseq = [0]*(self.deg+1)
        curr = [0.0]*self.deg

        lseq[0] = self.cp[0]
        rseq[self.deg] = self.cp[self.deg]
        for i in range(self.deg):
            curr[i] = self.cp[i].towards(self.cp[i+1], t)
        for i in range(self.deg-1):
            lseq[i+1] = curr[0]
            rseq[self.deg-i-1] = curr[self.deg-i-1]
            for j in range(self.deg-1-i):
                curr[j] = curr[j].towards(curr[j+1], t)
                lseq[self.deg] = curr[0]
                rseq[0] = curr[0]

        return [lseq, rseq]


def plot_cp(cp):
    x = []; y = []
    for i in range(len(cp)):
        x.append(cp[i].x)
        y.append(cp[i].y)
    plot(x, y)


def plot_bezier(bezier, n):

    eps = NT(1)/n
    bezier_curve = np.zeros(shape=(n+1, 2))
    t = 0
    for i in range(n+1):
        p = bezier.get_point(t)
        t += eps
        bezier_curve[i, 0] = p.x
        bezier_curve[i, 1] = p.y
    plot(bezier_curve[:,0], bezier_curve[:,1], 'ro')
    return bezier_curve


def pt(x, y):
    return Point(NT(x), NT(y))


def demo(vp):

    #this next curve has a singularity:
    #vp = [pt(0,0), pt(1,0), pt(.5, -.5), pt(.5, .5)]

    bc = Bezier(vp)


        #plotCP(vp)
    bezier_curve = plot_bezier(bc, 100)
    print bezier_curve
    
    show()

    return bezier_curve

if __name__ == "__main__":
    a=1
    b=1
    c=0.6
    vp = [pt(0, 0), pt(0.6, 0), pt(a, b), pt(a, 2*b), pt(a, 2*b+5)]
   # vp = [pt(0, 0), pt(0.6, 0), pt(a, b), pt((2*a)-c, 2*b), pt(2*a, 2*b)]
    #vp = [pt(0, 0), pt(0.6, 0), pt(a, b)]
    #vp = [pt(0, 0), pt(0.6, 0), pt(a, b)]
    demo(vp)