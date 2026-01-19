import cv2
import rospy
from std_msgs.msg import String
from typing import List, Tuple
from yolov5facedetector.face_detector import YoloDetector


class FaceResult:
    """단일 얼굴 결과"""

    def __init__(
        self,
        bbox: List[int],
        score: float,
        landmarks: List[Tuple[int, int]],
    ):
        self.bbox = bbox          # [x1, y1, x2, y2]
        self.score = score        # confidence
        self.landmarks = landmarks  # [(x, y), ...]

class RosTopicPublish:
    def __init__(self):
        self.data = ""
        self.pub = rospy.Publisher(
            "/face_direction",
            String,
            queue_size=10
        )

    def topicpub(self, data):
        msg = String()
        msg.data = data
        self.pub.publish(msg)

class FaceDirection:
    def __init__(self):
        self.topic = RosTopicPublish()

    def movedirection(self, frame, fx, fy):
        frame_x = frame.shape[1] / 20
        frame_y = frame.shape[0] / 20
        data = None

        if fx < -frame_x:
            print('left')
            data = "left"
            self.topic.topicpub(data)
        elif fx > frame_x:
            print('right')
            data = "right"
            self.topic.topicpub(data)
            

        if fy < -frame_y:
            print('down')
            data = "down"
            self.topic.topicpub(data)
        elif fy > frame_y:
            print('up')
            data = "up"
            self.topic.topicpub(data)

class FirstFace:
    def firstface(frame, face):
        frame_cx = frame.shape[1] / 2
        frame_cy = frame.shape[0] / 2

        # 가장 큰 얼굴 찾기
        max_area = 0
        best_bbox = None

        for face in face:
            x1, y1, x2, y2 = face.bbox
            area = (x2 - x1) * (y2 - y1)

            if area > max_area:
                max_area = area
                best_bbox = (x1, y1, x2, y2)

        # 선택된 얼굴 중심
        x1, y1, x2, y2 = best_bbox
        x = x1 + (x2 - x1) / 2
        y = y1 + (y2 - y1) / 2

        # 프레임 중심 기준 오프셋
        fx = frame_cx - x
        fy = frame_cy - y

        return fx, fy

class FaceDetector:
    """YOLOv5 Face Detector 래퍼"""

    def __init__(self):
        self.detector = YoloDetector()

    def detect(self, frame) -> List[FaceResult]:
        """
        yolov5facedetector 반환값 구조:
        [
          [bboxes],
          [scores],
          [landmarks]
        ]
        """
        results = self.detector(frame)

        bboxes = results[0][0]
        scores = results[1][0]
        landmarks = results[2][0]

        # faces: List[FaceResult] = []
        faces = []

        for bbox, score, lm in zip(bboxes, scores, landmarks):
            face = FaceResult(
                bbox=bbox,
                score=float(score[0]),
                landmarks=[tuple(p) for p in lm],
            )
            faces.append(face)

        return faces


class WebcamFaceApp:
    """웹캠 얼굴 인식 애플리케이션"""

    def __init__(self, device: str = "/dev/mywebcam"):
        self.cap = cv2.VideoCapture(device)

        self.firstface = FirstFace
        self.facedirection = FaceDirection()
        self.detector = FaceDetector()

        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 열기 실패: {device}")

    def draw_faces(self, frame, faces: List[FaceResult]):
        # print(len(faces))
        fx, fy = self.firstface.firstface(frame, faces)
        print(fx, fy)
        self.facedirection.movedirection(frame, fx, fy)

        for face in faces:
            x1, y1, x2, y2 = face.bbox

            # 얼굴 박스
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # confidence
            cv2.putText(
                frame,
                f"{face.score:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

            # landmarks (눈, 코, 입)
            for (lx, ly) in face.landmarks:
                cv2.circle(frame, (lx, ly), 2, (0, 0, 255), -1)

    def run(self):
        print("Webcam face detection started (press 'q' to quit)")

        while True:
            ret, frame = self.cap.read()
            # h, w = frame.shape[:2]
            # print(h, w)
            if not ret:
                print("프레임 읽기 실패")
                break

            faces = self.detector.detect(frame)
            self.draw_faces(frame, faces)

            cv2.imshow("Face Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()

class ImageFaceApp:

    def __init__(self, image_path: str):
        self.image_path = image_path
        self.detector = FaceDetector()

    def draw_faces(self, frame, faces: List[FaceResult]):
        for face in faces:
            x1, y1, x2, y2 = face.bbox

            # 얼굴 박스
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # confidence
            cv2.putText(
                frame,
                f"{face.score:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

            # landmarks (눈, 코, 입)
            for (lx, ly) in face.landmarks:
                cv2.circle(frame, (lx, ly), 2, (0, 0, 255), -1)

    def run(self):
        image = cv2.imread(self.image_path)
        if image is None:
            raise RuntimeError(f"이미지 로드 실패: {self.image_path}")

        faces = self.detector.detect(image)
        self.draw_faces(image, faces)

        print(f" Detected faces: {len(faces)}")

        cv2.imshow("Image Face Detection", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("face_direction_node", anonymous=True)

    # image = ImageFaceApp("yolov5/data/images/face.jpg")
    # image.run()

    cam = WebcamFaceApp("/dev/mywebcam")
    cam.run()