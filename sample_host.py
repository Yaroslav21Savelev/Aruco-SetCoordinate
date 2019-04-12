import conLib as connect;
import cv2

session = connect.host(4219)
session.accept()

while True:
    ret, frame = session.read()
    cv2.imshow("cap", frame)
    
    session.write((',0,,0' + ',\n').zfill(20).encode())
    if cv2.waitKey(1) == ord('q'):
        break
cv2.destroyAllWindows()
session.close() 