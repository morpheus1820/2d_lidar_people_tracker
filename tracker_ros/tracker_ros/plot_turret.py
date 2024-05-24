import csv
import cv2
import numpy as np
import sys


counter = 0
scale = 10
imgzero = np.zeros((512,512), dtype=np.uint8)
heatmap = np.zeros((512,512), dtype=np.float32)
heatmapzero = np.zeros((512,512), dtype=np.float32)
heatmap_windowing = True

out = cv2.VideoWriter(sys.argv[1].split('/')[-1] + '_heatmap.avi',  
                         cv2.VideoWriter_fourcc(*'MJPG'), 
                         30, (512,512)) 
                         
with open(sys.argv[1], 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in spamreader:
        n_poses = int(row[2])
        if n_poses > 0:
            xx = [float(x) for x in row[3::2]]
            yy = [float(x) for x in row[4::2]]            
            img = np.copy(imgzero)
                       
            for x,y in zip(xx,yy):
                x = int(x*scale) + 200
                y = int(y*scale) + 200

                cv2.circle(img, (x,y), 4, (255,0,0), -1)
                heatmap[x,y] += 0.01
               
            heatmap_blurred = cv2.GaussianBlur(heatmap, (15, 15), 0)
            heatmap_norm = cv2.normalize(heatmap_blurred, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            heatmap_color = cv2.applyColorMap(heatmap_norm, cv2.COLORMAP_JET)
            heatmap_color = cv2.putText(heatmap_color, row[0] + " " +row[1], (10,20), cv2.FONT_HERSHEY_SIMPLEX,  
                               0.5, (255,255,255), 1, cv2.LINE_AA) 

            cv2.imshow('heatmap', heatmap_color)
            c = cv2.waitKey(10)
            if c == 27:
                break
                
            out.write(heatmap_color)

            # decrease heatmap overall
            if heatmap_windowing and  counter % 100 == 0:
                heatmap = np.maximum(heatmapzero, heatmap - 0.01)

        counter += 1
        
out.release()
cv2.destroyAllWindows()
