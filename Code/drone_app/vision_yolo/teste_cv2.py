import cv2
import numpy as np

img = np.zeros((500, 500, 3), dtype=np.uint8)

while True:

    cv2.putText(
        img,
        "OPEN CV FUNCIONANDO",
        (40, 250),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255,255,255),
        2
    )

    cv2.imshow("TESTE", img)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
