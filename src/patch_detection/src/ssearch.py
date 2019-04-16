import cv2
import numpy as np
from sklearn.linear_model import LogisticRegression
 
def selectsearch(im,model):
    # speed-up using multithreads
    cv2.setUseOptimized(True);
    cv2.setNumThreads(16);
 
    # read image
    # im = cv2.imread("../test2.jpg")
    # resize image
    newHeight = 200
    newWidth = int(im.shape[1]*200/im.shape[0])
    # im = cv2.resize(im, (newWidth, newHeight))    
    scale = 0.1
    scbig = int(1/scale)
    im_ori = im.copy()
    im = cv2.resize(im, (int(scale*im.shape[1]), int(scale*im.shape[0])))    
     
    # create Selective Search Segmentation Object using default parameters
    ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()
    # ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentationStrategyColor()

 
    # set input image on which we will run segmentation
    ss.setBaseImage(im)
 
    # Switch to fast but low recall Selective Search method

    ss.switchToSelectiveSearchFast()
    # ss.switchToSelectiveSearchQuality()
 
    # run selective search segmentation on input image
    rects = ss.process()
    # print('Total Number of Region Proposals: {}'.format(len(rects)))
     
    # number of region proposals to show
    numShowRects = 20
    region_proposal = []
    max_prob = 0
    for i, rect in enumerate(rects):
    # draw rectangle for region proposal till numShowRects
        if (i < numShowRects):
            x, y, w, h = scbig*rect
            # cv2.rectangle(im_ori, (x, y), (x+w, y+h), (0, 255, 0), 1, cv2.LINE_AA)
            region_proposal.append(cv2.resize(im_ori[y:y+h,x:x+w],(256,256)).reshape(1,-1))
        else:
            break
    region_proposal = np.vstack(region_proposal)
    best_rect = rects[np.argmax(model.predict_proba(region_proposal)[:,1])]
    x,y,w,h = scbig*best_rect
    cv2.rectangle(im_ori, (x, y), (x+w, y+h), (255, 0, 0), 10, cv2.LINE_AA)
    return im_ori,best_rect

if __name__ == '__main__':
    im = cv2.imread("../test2.jpg")
    image = selectsearch(im)
    cv2.imshow("Output", image)
    cv2.waitKey(0)

