"""手动配准：可见光与热成像特征点选点配准与融合"""
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

# 图像路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = os.path.join(SCRIPT_DIR, 'single_frame_pic')
VIS_PATH = os.path.join(IMG_DIR, 'visible.jpg')
THERMAL_PATH = os.path.join(IMG_DIR, 'thermal.jpg')

MIN_POINTS = 4


def _imread(path):
    """读取图像，兼容中文路径"""
    try:
        with open(path, 'rb') as f:
            data = np.frombuffer(f.read(), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception:
        return None


RANSAC_THRESH = 5.0
CANNY_LOW, CANNY_HIGH = 50, 150
OVERLAY_VIS, OVERLAY_INFRA = 0.6, 0.4


def _on_click(event, x, y, flags, param):
    """鼠标点击回调：在图上标记点并记录坐标"""
    img, points, win_name = param
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(img, str(len(points)), (x + 7, y + 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(win_name, img)
        print(f"[{win_name}] 第 {len(points)} 点: ({x}, {y})")


def _select_points(img, win_name, points, enable_zoom=False):
    """在图像上交互选点，完成后按任意键继续。enable_zoom 时显示放大滑块"""
    img_disp = img.copy()
    cv2.namedWindow(win_name)

    if enable_zoom:
        state = {'img': img_disp, 'points': points, 'win_name': win_name, 'scale': 1.0}

        def _on_trackbar(val):
            state['scale'] = 1.0 + val / 100.0  # 0~150 -> 100%~250%
            disp = cv2.resize(
                state['img'], None, fx=state['scale'], fy=state['scale'],
                interpolation=cv2.INTER_LINEAR
            )
            cv2.imshow(win_name, disp)

        def _on_click_zoom(e, x, y, f, p):
            if e == cv2.EVENT_LBUTTONDOWN:
                s = p['scale']
                x_orig = int(np.clip(x / s, 0, p['img'].shape[1] - 1))
                y_orig = int(np.clip(y / s, 0, p['img'].shape[0] - 1))
                p['points'].append((x_orig, y_orig))
                cv2.circle(p['img'], (x_orig, y_orig), 5, (0, 0, 255), -1)
                cv2.putText(p['img'], str(len(p['points'])), (x_orig + 7, y_orig + 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                _on_trackbar(int((p['scale'] - 1.0) * 100))
                print(f"[{p['win_name']}] 第 {len(p['points'])} 点: ({x_orig}, {y_orig})")

        cv2.createTrackbar('放大', win_name, 0, 150, _on_trackbar)  # 100%~250%
        cv2.setMouseCallback(win_name, _on_click_zoom, state)
        _on_trackbar(0)
    else:
        cv2.setMouseCallback(win_name, _on_click, (img_disp, points, win_name))
        cv2.imshow(win_name, img_disp)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def _edge_fusion(img_vis, aligned_infra):
    """可见光打底 + 热红外边缘叠加"""
    gray = cv2.cvtColor(aligned_infra, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)
    edges = cv2.dilate(edges, np.ones((2, 2), np.uint8), iterations=1)
    result = img_vis.copy()
    result[edges > 0] = aligned_infra[edges > 0]
    return result


def manual_registration(vis_path=None, infra_path=None):
    """手动选点配准并融合，支持 4 点及以上"""
    vis_path = vis_path or VIS_PATH
    infra_path = infra_path or THERMAL_PATH

    img_vis = _imread(vis_path)
    img_infra = _imread(infra_path)
    if img_vis is None or img_infra is None:
        print("错误：无法读取图片，请检查路径。")
        print(f"  可见光: {vis_path}")
        print(f"  热成像: {infra_path}")
        return

    points_vis, points_infra = [], []

    # Step 1: 热成像选点
    print("请在【热成像图】窗口点击至少 4 个特征点，完成后按任意键...")
    _select_points(img_infra, "Step 1: 热成像 (选至少4点后按任意键)", points_infra)
    # Step 2: 可见光按相同顺序选点（支持放大）
    _select_points(img_vis, f"Step 2: 可见光 (按序点{len(points_infra)}个点)", points_vis, enable_zoom=True)

    if len(points_infra) < MIN_POINTS or len(points_vis) != len(points_infra):
        print(f"错误：点数不匹配或少于{MIN_POINTS}个。热成像:{len(points_infra)}, 可见光:{len(points_vis)}")
        return

    # 计算单应性矩阵并配准
    pts_dst = np.array(points_vis, dtype=float)
    pts_src = np.array(points_infra, dtype=float)
    M, _ = cv2.findHomography(pts_src, pts_dst, cv2.RANSAC, RANSAC_THRESH)
    print("单应性矩阵 H:\n", M)

    h, w = img_vis.shape[:2]
    aligned_infra = cv2.warpPerspective(img_infra, M, (w, h))

    # 融合结果
    overlay_classic = cv2.addWeighted(img_vis, OVERLAY_VIS, aligned_infra, OVERLAY_INFRA, 0)
    edge_fusion = _edge_fusion(img_vis, aligned_infra)

    # 拼接展示与保存
    infra_resized = cv2.resize(img_infra, (w, h))
    final_strip = cv2.hconcat([img_vis, infra_resized, overlay_classic, edge_fusion])

    out_path = os.path.join(SCRIPT_DIR, 'registration_final_result.jpg')
    cv2.imwrite(out_path, final_strip)
    print(f"已保存: {out_path}")

    # matplotlib 展示
    plt.figure(figsize=(10, 10))
    titles = ['Original Visible', 'Original Thermal', 'Classic Overlay', 'Infra Edge on Vis']
    imgs = [img_vis, infra_resized, overlay_classic, edge_fusion]
    for i in range(4):
        plt.subplot(2, 2, i + 1)
        plt.imshow(cv2.cvtColor(imgs[i], cv2.COLOR_BGR2RGB))
        plt.title(titles[i])
        plt.axis('off')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("--- 手动配准启动，请在弹出的窗口中选点 ---")
    manual_registration()
