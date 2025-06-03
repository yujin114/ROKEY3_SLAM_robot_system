#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from collections import deque

class MultiLevelBFS(Node):
    def __init__(self):
        super().__init__('multi_bfs_pathfinder')

        # ─── 1) YAML 파일 로드 ───────────────────────────────────────────────────────────
        try:
            # 1-1) World-level 트리 로드
            world_yaml = os.path.join(
                get_package_share_directory('rokey_pjt'),
                'config',
                'waypoints_world.yaml'
            )
            with open(world_yaml, 'r') as f:
                world_data = yaml.safe_load(f)
            self.world_graph = {}
            # world_graph 구조: { node_id: {'map_id': 'P1', 'entry_wp': 'p1_0', 'exit_wp': 'p1_2', 'neighbors': ['P2', ...]} }
            for wp in world_data['waypoints']:
                self.world_graph[wp['id']] = {
                    'map_id': wp['map_id'],
                    'entry_wp': wp['entry_wp'],
                    'exit_wp': wp['exit_wp'],
                    'neighbors': wp.get('neighbors', [])
                }

            # 1-2) Local-level 트리들 로드 (P1, P2 등)
            self.local_graphs = {}  # ex. {'P1': {'p1_0': ['p1_1'], 'p1_1': ['p1_2'], ...}, 'P2': {...}}
            for map_id in ['P1', 'P2']:
                yaml_path = os.path.join(
                    get_package_share_directory('rokey_pjt'),
                    'config',
                    f'waypoints_{map_id.lower()}.yaml'  # 예: waypoints_p1.yaml
                )
                with open(yaml_path, 'r') as f:
                    local_data = yaml.safe_load(f)
                graph = {}
                for wp in local_data['waypoints']:
                    graph[wp['id']] = wp.get('neighbors', [])
                # 무향 그래프로 만들려면 아래처럼 반대 간선도 추가 (선택 사항)
                for node, neighbors in graph.items():
                    for nbr in neighbors:
                        graph.setdefault(nbr, [])
                        if node not in graph[nbr]:
                            graph[nbr].append(node)
                self.local_graphs[map_id] = graph

            self.get_logger().info("Loaded world_graph and local_graphs successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML files: {e}")
            self.world_graph = {}
            self.local_graphs = {}

        # ─── 2) 상태 변수 ───────────────────────────────────────────────────────────────
        self.current_world_id = None
        self.goal_world_id = None

        # ─── 3) Subscriber & Publisher ─────────────────────────────────────────────────
        self.create_subscription(String, '/robot0/bfs/state_pose', self.state_callback, 10)
        self.create_subscription(String, '/robot0/bfs/goal_pose', self.goal_callback, 10)

        self.path_pub = self.create_publisher(String, '/robot0/bfs/path', 10)
        # (선택) 맵 스위칭 시점을 알리기 위한 퍼블리셔
        self.map_switch_pub = self.create_publisher(String, '/robot0/bfs/map_switch', 10)

        self.get_logger().info("MultiLevelBFS Node Initialized.")

    def state_callback(self, msg: String):
        self.current_world_id = msg.data.strip()
        self.get_logger().info(f"[state_callback] Current world: {self.current_world_id}")
        self.try_compute_path()

    def goal_callback(self, msg: String):
        self.goal_world_id = msg.data.strip()
        self.get_logger().info(f"[goal_callback] Goal world: {self.goal_world_id}")
        self.try_compute_path()

    def try_compute_path(self):
        # 둘 다 설정되어 있을 때만 실행
        if not self.current_world_id or not self.goal_world_id:
            return

        if self.current_world_id == self.goal_world_id:
            self.get_logger().warn("Current and Goal world are the same. No hierarchical path needed.")
            # 월드가 같으면 단일 로컬 맵 내에서 entry → exit BFS 만 계산
            single_map_id = self.world_graph[self.current_world_id]['map_id']
            entry = self.world_graph[self.current_world_id]['entry_wp']
            exit_wp = self.world_graph[self.current_world_id]['exit_wp']
            local_path = self.bfs_local(single_map_id, entry, exit_wp)
            path_str = ','.join(local_path) if local_path else ""
            self.path_pub.publish(String(data=path_str))
            return

        # 1) 월드 레벨 BFS
        world_path = self.bfs_world(self.current_world_id, self.goal_world_id)
        if not world_path:
            self.get_logger().warn(f"No world-level path from {self.current_world_id} to {self.goal_world_id}")
            self.path_pub.publish(String(data=""))
            return

        self.get_logger().info(f"World-level path: {world_path}")

        # 2) 얻어진 world_path를 순회하며 로컬 BFS 및 맵 스위칭 처리
        full_path = []
        for idx, world_node in enumerate(world_path):
            map_id = self.world_graph[world_node]['map_id']
            entry_wp = self.world_graph[world_node]['entry_wp']
            exit_wp = self.world_graph[world_node]['exit_wp']

            # 로컬 BFS: entry → exit
            local_segment = self.bfs_local(map_id, entry_wp, exit_wp)
            if not local_segment:
                self.get_logger().warn(f"[{map_id}] Local path not found: {entry_wp} → {exit_wp}")
                continue

            # 로컬 분할 경로를 full_path에 추가
            # (연결될 때 중복 제거: 이전 exit와 같은 값이 중복되므로 마지막 노드를 제외하고 병합)
            if full_path and full_path[-1] == local_segment[0]:
                full_path.extend(local_segment[1:])
            else:
                full_path.extend(local_segment)

            # 맵 스위칭: 마지막 노드(exit_wp)에 도달 시 다음 맵으로 스위칭
            # 단, 마지막 world_node가 goal_world_id라면 스위칭할 필요 없음
            if idx < len(world_path) - 1:
                next_world = world_path[idx+1]
                next_map = self.world_graph[next_world]['map_id']
                switch_msg = f"{map_id}->{next_map}"
                self.get_logger().info(f"Map switch demanded: {switch_msg}")
                self.map_switch_pub.publish(String(data=switch_msg))
                # 여기서 실제 nav2 맵 스위칭 코드(서비스 콜 또는 launch 파일 재시작 등)를  
                # 다른 콜백/노드가 감지해서 처리해야 한다.  
                # (예: map_switch_listener 노드가 이 토픽을 구독해 nav2에서 map 변경 등)

        # 3) 완성된 full_path 퍼블리시
        path_str = ','.join(full_path)
        self.get_logger().info(f"Full hierarchical path: {path_str}")
        self.path_pub.publish(String(data=path_str))

    def bfs_world(self, start_id, goal_id):
        """월드 그래프 상에서 BFS"""
        visited = set([start_id])
        queue = deque([(start_id, [start_id])])

        while queue:
            current, path = queue.popleft()
            if current == goal_id:
                return path
            for neighbor in self.world_graph[current]['neighbors']:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None

    def bfs_local(self, map_id, start_wp, goal_wp):
        """지정된 로컬 그래프에서 BFS"""
        graph = self.local_graphs.get(map_id, {})
        visited = set([start_wp])
        queue = deque([(start_wp, [start_wp])])

        while queue:
            current, path = queue.popleft()
            if current == goal_wp:
                return path
            for neighbor in graph.get(current, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None

def main(args=None):
    rclpy.init(args=args)
    node = MultiLevelBFS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
