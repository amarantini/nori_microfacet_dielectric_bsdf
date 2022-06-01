import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
 
# Read Images
img = mpimg.imread('table_mis1024_final.png')

width,height,_ = img.shape

img_out = np.zeros((width,height,3))
sigma_p = 5
k = 3
for i in range(width):
    for j in range(height):
        samples  = []
        for c in range(3):
            for m in range(i-2*sigma_p,i+2*sigma_p+1):
                for n in range(j-2*sigma_p,j+2*sigma_p+1):
                    if m>=0 and m<width and n>=0 and n<height:
                        samples.append(img[m][n][c])
        
            mean = np.mean(np.array(samples))
            std = np.std(np.array(samples))
            lower = max(mean-k*std,0)
            upper = min(mean+k*std,1)
    
            img_out[i][j][c] = max(lower , min(img[i][j][c], upper))
 

 
# Output Images

mpimg.imsave("table_mis1024_final_filtered.png", img_out)
