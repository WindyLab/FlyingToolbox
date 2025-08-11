import cv2
import os

# 读取图片
image_path = os.path.join(os.path.dirname(__file__), 'images', 'marker.png')
image = cv2.imread(image_path)

if image is None:
    print("无法读取图片，请检查路径是否正确")
else:
    # 显示图片
    cv2.imshow("Detected Marker", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 保存处理后的图片
    output_path = os.path.join(os.path.dirname(__file__), 'images', 'detected_marker.png')
    cv2.imwrite(output_path, image)
    print(f"图片已保存至: {output_path}")