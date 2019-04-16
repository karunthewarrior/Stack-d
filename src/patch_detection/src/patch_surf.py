#https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_matcher/py_matcher.html#flann-based-matcher
#https://www.learnopencv.com/selective-search-for-object-detection-cpp-python/
import cv2
import numpy as np
from sklearn.cluster import KMeans
from matplotlib import pyplot as plt
import scipy.ndimage


def surfing(im1,im2):
    im1 = scipy.ndimage.gaussian_filter(im1,sigma=0.5)
    im1 = cv2.resize(im1,(250,250))
    im2 = cv2.imread("../data/end4.jpg")
    im2 = scipy.ndimage.gaussian_filter(im2,sigma=0.5)

    locs1, desc1 = surf.detectAndCompute(im1,None)
    locs2, desc2 = surf.detectAndCompute(im2,None)
    bf = cv2.BFMatcher(cv2.NORM_L2)
    matches = bf.knnMatch(desc1,desc2,k=2)
    x = np.eye(800)
    good = []
    for m,n in matches:
        if m.distance < 0.9*n.distance:
            good.append([m])
    print(matches)
    img3 = cv2.drawMatchesKnn(im1,locs1,im2,locs2,good,x, flags=2)

def surf(im2):
    scale = 0.3
    im2 = cv2.resize(im2,(int(scale*im2.shape[0]),int(scale*im2.shape[1])))
    surf = cv2.xfeatures2d.SURF_create(900)

    locs2, desc2 = surf.detectAndCompute(im2,None)
    print("Matching")
    print(model.shape)
    bf = cv2.BFMatcher(cv2.NORM_L2)
    matches = bf.knnMatch(model,desc2,k=2)
    x = np.eye(800)
    list_kp2 = np.array([locs2[mat.trainIdx].pt for mat,n in matches if mat.distance < 0.8*n.distance])
    main_dot = np.array([np.mean(list_kp2[:,0]),np.mean(list_kp2[:,1])])
    return list_kp2,main_dot


if __name__=='__main__':
    model = np.load("train_desc.npy")
    scale = 0.3
    im2 = cv2.imread("../data/test5.jpg")
    im2 = scipy.ndimage.gaussian_filter(im2,sigma=0.5)
    im2 = cv2.resize(im2,(int(scale*im2.shape[0]),int(scale*im2.shape[1])))
    surf = cv2.xfeatures2d.SURF_create(900)

    locs2, desc2 = surf.detectAndCompute(im2,None)
    print("Matching")
    print(model.shape)
    bf = cv2.BFMatcher(cv2.NORM_L2)
    matches = bf.knnMatch(model,desc2,k=2)
    x = np.eye(800)
    list_kp2 = np.array([locs2[mat.trainIdx].pt for mat,n in matches if mat.distance < 0.8*n.distance])
    main_dot = np.array([np.mean(list_kp2[:,0]),np.mean(list_kp2[:,1])])
    print(main_dot)


    # img3 = cv2.drawMatchesKnn(im1,locs1,im2,locs2,matches,x, flags=2)
    # plt.imshow(img3),plt.show()
    plt.imshow(im2)
    plt.plot(list_kp2[:,0],list_kp2[:,1],'og',markersize=10)
    plt.plot(main_dot[0],main_dot[1],'or',markersize=12)
    plt.show()
