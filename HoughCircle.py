import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('car.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 灰度图像
# plt.imshow(gray)
plt.subplot(121), plt.imshow(img, cmap='gray')
plt.title('img'), plt.xticks([]), plt.yticks([])
circle1 = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=100, minRadius=30, maxRadius=150)  #把半径范围缩小点，检测内圆，瞳孔
print(circle1)
circles = circle1[0, :, :]  # 提取为二维
circles = np.uint16(np.around(circles))  # 四舍五入，取整
for i in circles[:]:
    cv2.circle(img, (i[0], i[1]), i[2], (255, 0, 0), 5)  # 画圆
    cv2.circle(img, (i[0], i[1]), 2, (255, 0, 0), 10)  # 画圆心

plt.subplot(122), plt.imshow(img)
plt.title('circle'), plt.xticks([]), plt.yticks([])
plt.show()