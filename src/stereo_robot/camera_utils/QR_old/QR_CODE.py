import cv2


# Display barcode and QR_old code location
def display(im, bbox):
    bbox = np.array(bbox,dtype=int)
    n = len(bbox)
    for j in range(n):
        print(j)
        arg1 = tuple(bbox[j][0])
        arg2 = tuple(bbox[ (j+1) % n][0])
        
        cv2.line(im, arg1, arg2, (255,0,0), 3)

    # Display results
    cv2.imshow("Results", im)

import numpy as np
from pyzbar import pyzbar
if __name__=='__main__':
    cam= cv2.VideoCapture(0)
    cv2.namedWindow('Results')
    while True:
        ret,frame = cam.read()
        
        barcodes = pyzbar.decode(frame)
        for barcode in barcodes:
            (x,y,w,h) = barcode.rect
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 2)
            
        cv2.imshow("Results",frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
    
    cv2.destroyAllWindows()    
