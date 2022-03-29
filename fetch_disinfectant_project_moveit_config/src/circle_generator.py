import numpy as np
import matplotlib.pyplot as plt

n = [1, 6, 12]
r = [0.0, 0.025, 0.05]


# def rtpairs(r, n):
#
#     for i in range(len(r)):
#        for j in range(n[i]):
#         yield r[i], j*(2 * np.pi / n[i])
#
# for r, t in rtpairs(R, T):
#     plt.plot(r * np.cos(t), r * np.sin(t), 'bo')
# plt.show()

def circle_points(r, n):
    circles = []
    for r, n in zip(r, n):
        t = np.linspace(0, 2*np.pi, n, endpoint=False)
        x = r * np.cos(t)
        y = r * np.sin(t)
        z = [-0.3]*len(x)

        circles.append(np.c_[x, y, z])
        # concatenate = np.concatenate( circles, axis=0 )
    return circles

circles = circle_points(r, n)
# print(np.shape(circles[0]))
# print(np.shape(circles[1]))
# print(np.shape(circles[2]))
print(circles)
fig, ax = plt.subplots()
for circle in circles:
    ax.scatter(circle[:, 0], circle[:, 1])
ax.set_aspect('equal')
plt.show()
