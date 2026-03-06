import json
import matplotlib.pyplot as plt
import argparse
import os
import numpy as np

def douglas_peucker(points, epsilon):
    """경로 특징 유지하며 점 개수 줄이기"""
    if len(points) < 3: return points
    dmax = 0
    index = 0
    for i in range(1, len(points) - 1):
        d = np.abs(np.cross(points[-1] - points[0], points[0] - points[i])) / np.linalg.norm(points[-1] - points[0])
        if d > dmax:
            index = i
            dmax = d
    if dmax > epsilon:
        res1 = douglas_peucker(points[:index+1], epsilon)
        res2 = douglas_peucker(points[index:], epsilon)
        return np.vstack((res1[:-1], res2))
    else:
        return np.vstack((points[0], points[-1]))

def resample_path(x, y, interval=0.01):
    """점 사이를 일정한 간격으로 촘촘하게 채우기 (보간)"""
    new_x, new_y = [], []
    for i in range(len(x) - 1):
        p1 = np.array([x[i], y[i]])
        p2 = np.array([x[i+1], y[i+1]])
        dist = np.linalg.norm(p2 - p1)

        # 구간당 필요한 점의 개수 계산
        num_points = max(1, int(dist / interval))
        interp_x = np.linspace(p1[0], p2[0], num_points, endpoint=False)
        interp_y = np.linspace(p1[1], p2[1], num_points, endpoint=False)

        new_x.extend(interp_x.tolist())
        new_y.extend(interp_y.tolist())

    # 마지막 점 추가
    new_x.append(x[-1])
    new_y.append(y[-1])
    return new_x, new_y

class SmartPathEditor:
    def __init__(self, input_path, output_path):
        self.input_path = input_path
        self.output_path = output_path
        if not self.load_data(): return

        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.line, = self.ax.plot(self.x, self.y, 'b-', alpha=0.5, zorder=1)
        self.points_plot, = self.ax.plot(self.x, self.y, 'ro', markersize=3, picker=5, zorder=2)

        self.selected_index = None
        self.update_ui()

        # 이벤트 연결
        self.fig.canvas.mpl_connect('pick_event', self.on_pick)
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)

        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def load_data(self):
        try:
            with open(self.input_path, 'r') as f:
                data = json.load(f)
            self.x, self.y = list(data['X']), list(data['Y'])
            return True
        except Exception as e:
            print(f"Error: {e}"); return False

    def update_ui(self):
        self.ax.set_title(f"Points: {len(self.x)} | [P: Simplify] [R: Resample(Dense)] [S: Save]\n[RightClick: Delete] [Double-Click Line: Add]")
        self.line.set_data(self.x, self.y)
        self.points_plot.set_data(self.x, self.y)
        self.fig.canvas.draw_idle()

    def on_key(self, event):
        if event.key.lower() == 'p': # 단순화 (편집하기 쉽게)
            pts = np.column_stack((self.x, self.y))
            simplified = douglas_peucker(pts, 0.05)
            self.x, self.y = simplified[:,0].tolist(), simplified[:,1].tolist()
            print(f"Simplified: {len(self.x)} points")
            self.update_ui()
        elif event.key.lower() == 'r': # 다시 촘촘하게 (0.01 간격)
            self.x, self.y = resample_path(self.x, self.y, interval=0.01)
            print(f"Resampled (Dense): {len(self.x)} points")
            self.update_ui()
        elif event.key.lower() == 's':
            with open(self.output_path, 'w') as f:
                json.dump({'X': self.x, 'Y': self.y}, f, indent=4)
            print(f"Saved to: {self.output_path}")

    # (이전 코드의 on_press, on_pick, on_motion, on_release, on_scroll 함수와 동일)
    def on_press(self, event):
        if event.inaxes != self.ax: return
        if event.dblclick and event.button == 1:
            dists = np.sqrt((np.array(self.x)-event.xdata)**2 + (np.array(self.y)-event.ydata)**2)
            idx = np.argmin(dists)
            self.x.insert(idx + 1, event.xdata); self.y.insert(idx + 1, event.ydata)
            self.update_ui()
        elif event.button == 3:
            dists = np.sqrt((np.array(self.x)-event.xdata)**2 + (np.array(self.y)-event.ydata)**2)
            if len(dists) > 0 and np.min(dists) < 0.2:
                idx = np.argmin(dists); self.x.pop(idx); self.y.pop(idx)
                self.update_ui()
    def on_pick(self, event):
        if event.artist == self.points_plot and self.fig.canvas.manager.toolbar.mode == '':
            self.selected_index = event.ind[0]
    def on_motion(self, event):
        if self.selected_index is not None and event.inaxes == self.ax:
            self.x[self.selected_index], self.y[self.selected_index] = event.xdata, event.ydata
            self.update_ui()
    def on_release(self, event): self.selected_index = None
    def on_scroll(self, event):
        if event.inaxes != self.ax: return
        scale = 0.8 if event.button == 'up' else 1.2
        cur_x, cur_y = self.ax.get_xlim(), self.ax.get_ylim()
        self.ax.set_xlim([event.xdata - (event.xdata-cur_x[0])*scale, event.xdata + (cur_x[1]-event.xdata)*scale])
        self.ax.set_ylim([event.ydata - (event.ydata-cur_y[0])*scale, event.ydata + (cur_y[1]-event.ydata)*scale])
        self.fig.canvas.draw_idle()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("-o", "--output")
    args = parser.parse_args()
    out = args.output if args.output else args.input.replace(".json", "_fixed.json")
    SmartPathEditor(args.input, out)

if __name__ == "__main__":
    main()
