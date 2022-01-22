import cv2
import color_filter
class ContourDetection:
    def __init__(self, image, minArea = 10):
        self.mask = image
        self.minArea = minArea

    def getContours(self):
        mask = self.mask
        height, width = mask.shape
        edges=None
        # for i in range(3,7, 2):
        edges = cv2.Canny(mask,100, 150, apertureSize=5)
        cv2.imshow("Canny", edges)
        _, contours, _= cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        newContours = []
        for contour in contours:
            if cv2.contourArea(contour)>self.minArea:
                if 3<self.getSideNumber(contour)<8:
                    newContours.append(contour)

        return newContours

    def getSideNumber(self, contour):
        epilson = 0.04*cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epilson, True)
        return len(approx)
    