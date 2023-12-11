import matplotlib.pyplot as plt
import math
ox, oy = [], []
deg = list(range(0, 360, 5))
#top wall
for i in range(0, 400):
    # ox.append(i)
    # oy.append(0.0)
    for d in deg:
        xl = i + 16 * math.cos(math.radians(d)) 
        yl = 0.0 + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 and yl == 16 and xl > 16 and xl < 384:
            ox.append(xl)
            oy.append(yl)
#right
for i in range(0, 150):
    # ox.append(400.0)
    # oy.append(i)
    for d in deg:
        xl = 400.0 + 16 * math.cos(math.radians(d)) 
        yl = i + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 and xl == 400 - 16 and yl > 16 and yl < 134:
            ox.append(xl)
            oy.append(yl)
for i in range(0, 401):
    # ox.append(i)
    # oy.append(150.0)
    for d in deg:
        xl = i + 16 * math.cos(math.radians(d)) 
        yl = 150.0 + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 and yl == 150 - 16 and xl > 16 and xl < 384:
            ox.append(xl)
            oy.append(yl)
for i in range(0, 151):
    # ox.append(0.0)
    # oy.append(i)
    for d in deg:
        xl = 0.0 + 16 * math.cos(math.radians(d)) 
        yl = i + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 and xl == 16 and yl > 16 and yl < 134:
            ox.append(xl)
            oy.append(yl)
#obstacles
#additional line segment
for i in range(0, 16):
    ox.append(50.0 - 16 + i)
    oy.append(99.0)
    ox.append(150.0 + i)
    oy.append(99.0)
    ox.append(200.0)
    oy.append(50.0 - 16 + i)
    ox.append(350.0 + i)
    oy.append(99.0)
    ox.append(250.0 - 16 + i)
    oy.append(99.0)
#left obstacle
for i in range(0, 100):
    if i > 15:
        ox.append(99.0)
        oy.append(i)
    for d in deg:
        xl = 99.0 + 16 * math.cos(math.radians(d)) 
        yl = i + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 and (xl == 99 + 16 or xl == 99 - 16) and yl > 16 and yl < 83:
            ox.append(xl)
            oy.append(yl)
for i in range(0, 100):
    ox.append(50.0 + i)
    oy.append(99.0)
    for d in deg:
        xl = 50.0 + i + 16 * math.cos(math.radians(d)) 
        yl = 99.0 + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 and yl == 99 + 16 or yl == 99 - 16:
            ox.append(xl)
            oy.append(yl)
#left half circle
deg2 = list(range(90, 270, 5))
for d2 in deg2:
    xl = 50.0 + 16 * math.cos(math.radians(d2)) 
    yl = 99.0 + 16 * math.sin(math.radians(d2))  
    ox.append(xl)
    oy.append(yl)
deg3 = list(range(90, -90, -5))
#right half circle
for d3 in deg3:
    xl = 150.0 + 16 * math.cos(math.radians(d3)) 
    yl = 99.0 + 16 * math.sin(math.radians(d3))  
    ox.append(xl)
    oy.append(yl)
#mid obstacles
for i in range(0, 100):
    if 50 + i < 150-16:
        ox.append(200.0)
        oy.append(50.0 + i)
    for d in deg:
        xl = 200.0 + 16 * math.cos(math.radians(d)) 
        yl = 50.0 + i + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 and (xl == 200 + 16 or xl == 200 - 16) and yl < 134:
            ox.append(xl)
            oy.append(yl)
deg4 = list(range(180, 360, 5))
for d4 in deg4:
    xl = 200.0 + 16 * math.cos(math.radians(d4)) 
    yl = 50.0 + 16 * math.sin(math.radians(d4))  
    ox.append(xl)
    oy.append(yl)            
ox.append(200.0)
oy.append(50.0 - 16)
#right obstacle
for i in range(0, 100):
    if i > 15:
        ox.append(299.0)
        oy.append(i)
    for d in deg:
        xl = 299.0 + 16 * math.cos(math.radians(d)) 
        yl = i + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 and (xl == 299 + 16 or xl == 299 - 16) and yl > 16 and yl < 83:
            ox.append(xl)
            oy.append(yl)
for i in range(0, 100):
    ox.append(250.0 + i)
    oy.append(99.0)
    for d in deg:
        xl = 250.0 + i + 16 * math.cos(math.radians(d)) 
        yl = 99.0 + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 and yl == 99 + 16 or yl == 99 - 16:
            ox.append(xl)
            oy.append(yl)
#left half circle
for d2 in deg2:
    xl = 250.0 + 16 * math.cos(math.radians(d2)) 
    yl = 99.0 + 16 * math.sin(math.radians(d2))  
    ox.append(xl)
    oy.append(yl)
#right half circle
for d3 in deg3:
    xl = 350.0 + 16 * math.cos(math.radians(d3)) 
    yl = 99.0 + 16 * math.sin(math.radians(d3))  
    ox.append(xl)
    oy.append(yl)                     
ox.append(250.0 - 16)
oy.append(99.0)
ox.append(349.0 + 16)
oy.append(99.0)

#plt.plot(ox, oy, ".k")
plt.plot(ox, oy, ".",color='gray')
plt.gca().invert_yaxis()
plt.show()