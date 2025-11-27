#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hybrid A* Search Visualization Tool

Purpose:
- Visualize Hybrid A* search tree
- Visualize Reed-Shepp paths
- Visualize final coarse path
- Help debug Hybrid A* search failures

Usage:
    python3 visualize_hybrid_astar.py [--log-dir csv_logs/hybrid_astar] [--run-id latest]
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse
import os
import glob


class HybridAStarVisualizer:
    def __init__(self, log_dir="csv_logs/hybrid_astar", run_id="latest"):
        """
        Initialize visualizer

        Args:
            log_dir: CSV log directory
            run_id: Run ID, "latest" means use the most recent one
        """
        self.log_dir = Path(log_dir)

        if run_id == "latest":
            # Find the latest run ID
            pattern = str(self.log_dir / "*_coarse_path.csv")
            files = glob.glob(pattern)
            if not files:
                raise ValueError(f"No log files found in {log_dir}")
            latest_file = max(files, key=os.path.getctime)
            self.run_id = Path(latest_file).stem.replace("_coarse_path", "")
        else:
            self.run_id = run_id

        print(f"Using run ID: {self.run_id}")

        # Load data
        self.load_data()

    def load_data(self):
        """Load all CSV data"""
        try:
            # Search nodes
            search_file = self.log_dir / f"{self.run_id}_search_nodes.csv"
            if search_file.exists():
                self.search_nodes = pd.read_csv(search_file)
                print(f"Loaded {len(self.search_nodes)} search nodes")
            else:
                self.search_nodes = None
                print(f"Search nodes file not found: {search_file}")

            # Reed-Shepp path points
            rs_points_file = self.log_dir / f"{self.run_id}_rs_path_points.csv"
            if rs_points_file.exists():
                self.rs_path_points = pd.read_csv(rs_points_file)
                print(f"Loaded {len(self.rs_path_points)} RS path points")
            else:
                self.rs_path_points = None
                print(f"RS path points file not found: {rs_points_file}")

            # Reed-Shepp path summary
            rs_summary_file = self.log_dir / f"{self.run_id}_rs_path_summary.csv"
            if rs_summary_file.exists():
                self.rs_path_summary = pd.read_csv(rs_summary_file)
                print(f"Loaded {len(self.rs_path_summary)} RS paths")
            else:
                self.rs_path_summary = None
                print(f"RS path summary file not found: {rs_summary_file}")

            # Coarse path
            coarse_file = self.log_dir / f"{self.run_id}_coarse_path.csv"
            if coarse_file.exists():
                self.coarse_path = pd.read_csv(coarse_file)
                print(f"Loaded {len(self.coarse_path)} coarse path points")
            else:
                self.coarse_path = None
                print(f"Coarse path file not found: {coarse_file}")

        except Exception as e:
            print(f"Error loading data: {e}")
            raise

    def plot_search_tree(self, max_nodes=5000, alpha=0.3):
        """
        Plot search tree

        Args:
            max_nodes: Maximum number of nodes to display
            alpha: Node transparency
        """
        if self.search_nodes is None:
            print("No search node data")
            return

        plt.figure(figsize=(12, 10))

        # Only display first max_nodes nodes
        nodes = self.search_nodes.head(max_nodes)

        # Classify by direction (forward/backward)
        forward = nodes[nodes['direction'] == 1]
        backward = nodes[nodes['direction'] == 0]

        # Plot nodes
        plt.scatter(forward['x'], forward['y'], c='blue', s=10, alpha=alpha, label='Forward nodes')
        plt.scatter(backward['x'], backward['y'], c='red', s=10, alpha=alpha, label='Backward nodes')

        # Plot direction arrows (show one arrow every 10 nodes)
        step = max(1, len(nodes) // 100)
        for i in range(0, len(nodes), step):
            row = nodes.iloc[i]
            dx = 0.5 * np.cos(row['phi'])
            dy = 0.5 * np.sin(row['phi'])
            color = 'blue' if row['direction'] == 1 else 'red'
            plt.arrow(row['x'], row['y'], dx, dy,
                     head_width=0.2, head_length=0.1, fc=color, ec=color, alpha=alpha)

        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title(f'Hybrid A* Search Tree (showing {len(nodes)}/{len(self.search_nodes)} nodes)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()

    def plot_cost_distribution(self):
        """Plot cost distribution"""
        if self.search_nodes is None:
            print("No search node data")
            return

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # Trajectory cost distribution
        axes[0, 0].hist(self.search_nodes['traj_cost'], bins=50, alpha=0.7, color='blue')
        axes[0, 0].set_xlabel('Trajectory Cost')
        axes[0, 0].set_ylabel('Frequency')
        axes[0, 0].set_title('Trajectory Cost Distribution')
        axes[0, 0].grid(True, alpha=0.3)

        # Heuristic cost distribution
        axes[0, 1].hist(self.search_nodes['heuristic_cost'], bins=50, alpha=0.7, color='green')
        axes[0, 1].set_xlabel('Heuristic Cost')
        axes[0, 1].set_ylabel('Frequency')
        axes[0, 1].set_title('Heuristic Cost Distribution')
        axes[0, 1].grid(True, alpha=0.3)

        # Total cost distribution
        axes[1, 0].hist(self.search_nodes['total_cost'], bins=50, alpha=0.7, color='red')
        axes[1, 0].set_xlabel('Total Cost')
        axes[1, 0].set_ylabel('Frequency')
        axes[1, 0].set_title('Total Cost Distribution')
        axes[1, 0].grid(True, alpha=0.3)

        # Steering angle distribution
        axes[1, 1].hist(self.search_nodes['steering'], bins=50, alpha=0.7, color='orange')
        axes[1, 1].set_xlabel('Steering Angle (rad)')
        axes[1, 1].set_ylabel('Frequency')
        axes[1, 1].set_title('Steering Angle Distribution')
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()

    def plot_reed_shepp_paths(self, show_all=False):
        """
        Plot Reed-Shepp paths

        Args:
            show_all: Whether to show all paths (if False, only show the last one)
        """
        if self.rs_path_points is None:
            print("No Reed-Shepp path data")
            return

        plt.figure(figsize=(12, 10))

        # Group by path ID
        paths = self.rs_path_points.groupby('path_id')

        if show_all:
            # Show all paths
            for path_id, path_data in paths:
                forward = path_data[path_data['gear'] == 1]
                backward = path_data[path_data['gear'] == 0]

                plt.plot(forward['x'], forward['y'], 'b-', alpha=0.5, linewidth=1)
                plt.plot(backward['x'], backward['y'], 'r-', alpha=0.5, linewidth=1)
        else:
            # Only show the last path (most likely to be successful)
            last_path_id = self.rs_path_points['path_id'].max()
            last_path = paths.get_group(last_path_id)

            forward = last_path[last_path['gear'] == 1]
            backward = last_path[last_path['gear'] == 0]

            if len(forward) > 0:
                plt.plot(forward['x'], forward['y'], 'b-', linewidth=2, label='Forward')
            if len(backward) > 0:
                plt.plot(backward['x'], backward['y'], 'r-', linewidth=2, label='Backward')

            # Show start and end points
            plt.plot(last_path.iloc[0]['x'], last_path.iloc[0]['y'], 'go', markersize=10, label='Start')
            plt.plot(last_path.iloc[-1]['x'], last_path.iloc[-1]['y'], 'ro', markersize=10, label='End')

        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Reed-Shepp Paths')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()

    def plot_coarse_path(self, show_velocity=True):
        """
        Plot coarse path

        Args:
            show_velocity: Whether to show velocity information
        """
        if self.coarse_path is None:
            print("No coarse path data")
            return

        if show_velocity:
            fig, axes = plt.subplots(2, 1, figsize=(12, 10))

            # Path plot
            ax1 = axes[0]
            sc = ax1.scatter(self.coarse_path['x'], self.coarse_path['y'],
                           c=self.coarse_path['v'], cmap='coolwarm', s=50)

            # Plot direction arrows
            step = max(1, len(self.coarse_path) // 20)
            for i in range(0, len(self.coarse_path), step):
                row = self.coarse_path.iloc[i]
                dx = 0.5 * np.cos(row['phi'])
                dy = 0.5 * np.sin(row['phi'])
                ax1.arrow(row['x'], row['y'], dx, dy,
                         head_width=0.2, head_length=0.1, fc='black', ec='black', alpha=0.5)

            ax1.plot(self.coarse_path.iloc[0]['x'], self.coarse_path.iloc[0]['y'],
                    'go', markersize=12, label='Start')
            ax1.plot(self.coarse_path.iloc[-1]['x'], self.coarse_path.iloc[-1]['y'],
                    'ro', markersize=12, label='End')

            plt.colorbar(sc, ax=ax1, label='Velocity (m/s)')
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_title('Coarse Path (color indicates velocity)')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            ax1.axis('equal')

            # Velocity and acceleration curves
            ax2 = axes[1]
            ax2.plot(self.coarse_path['index'], self.coarse_path['v'], 'b-', label='Velocity (m/s)')
            ax2.plot(self.coarse_path['index'], self.coarse_path['a'], 'r-', label='Acceleration (m/s²)')
            ax2.plot(self.coarse_path['index'], self.coarse_path['steer'], 'g-', label='Steering Angle (rad)')
            ax2.set_xlabel('Path Point Index')
            ax2.set_ylabel('Value')
            ax2.set_title('Velocity, Acceleration and Steering Angle')
            ax2.legend()
            ax2.grid(True, alpha=0.3)

        else:
            plt.figure(figsize=(12, 10))
            plt.plot(self.coarse_path['x'], self.coarse_path['y'], 'b-', linewidth=2)
            plt.plot(self.coarse_path.iloc[0]['x'], self.coarse_path.iloc[0]['y'],
                    'go', markersize=12, label='Start')
            plt.plot(self.coarse_path.iloc[-1]['x'], self.coarse_path.iloc[-1]['y'],
                    'ro', markersize=12, label='End')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title('Coarse Path')
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.axis('equal')

        plt.tight_layout()

    def plot_comprehensive(self):
        """Plot comprehensive view"""
        fig = plt.figure(figsize=(18, 12))
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)

        # 1. Search tree + RS paths + coarse path
        ax1 = fig.add_subplot(gs[0:2, 0])
        if self.search_nodes is not None:
            nodes = self.search_nodes.head(2000)
            ax1.scatter(nodes['x'], nodes['y'], c='gray', s=5, alpha=0.2, label='Search nodes')

        if self.rs_path_points is not None:
            last_path_id = self.rs_path_points['path_id'].max()
            last_path = self.rs_path_points[self.rs_path_points['path_id'] == last_path_id]
            ax1.plot(last_path['x'], last_path['y'], 'b--', linewidth=2, alpha=0.7, label='RS path')

        if self.coarse_path is not None:
            ax1.plot(self.coarse_path['x'], self.coarse_path['y'], 'r-', linewidth=3, label='Coarse path')
            ax1.plot(self.coarse_path.iloc[0]['x'], self.coarse_path.iloc[0]['y'],
                    'go', markersize=12, label='Start')
            ax1.plot(self.coarse_path.iloc[-1]['x'], self.coarse_path.iloc[-1]['y'],
                    'ro', markersize=12, label='End')

        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Hybrid A* Comprehensive View')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        # 2. Cost evolution
        ax2 = fig.add_subplot(gs[0, 1])
        if self.search_nodes is not None:
            ax2.plot(self.search_nodes['iteration'], self.search_nodes['total_cost'], 'b-', alpha=0.5)
            ax2.set_xlabel('Iteration')
            ax2.set_ylabel('Total Cost')
            ax2.set_title('Search Cost Evolution')
            ax2.grid(True, alpha=0.3)

        # 3. Velocity curve
        ax3 = fig.add_subplot(gs[1, 1])
        if self.coarse_path is not None:
            ax3.plot(self.coarse_path['index'], self.coarse_path['v'], 'b-', linewidth=2)
            ax3.set_xlabel('Path Point Index')
            ax3.set_ylabel('Velocity (m/s)')
            ax3.set_title('Velocity Curve')
            ax3.grid(True, alpha=0.3)

        # 4. Statistics
        ax4 = fig.add_subplot(gs[2, :])
        ax4.axis('off')

        stats_text = "Search Statistics:\n"
        if self.search_nodes is not None:
            stats_text += f"  - Total search nodes: {len(self.search_nodes)}\n"
            stats_text += f"  - Forward nodes: {len(self.search_nodes[self.search_nodes['direction']==1])}\n"
            stats_text += f"  - Backward nodes: {len(self.search_nodes[self.search_nodes['direction']==0])}\n"
            stats_text += f"  - Average trajectory cost: {self.search_nodes['traj_cost'].mean():.2f}\n"
            stats_text += f"  - Average heuristic cost: {self.search_nodes['heuristic_cost'].mean():.2f}\n"

        if self.rs_path_summary is not None:
            stats_text += f"\nReed-Shepp Paths:\n"
            stats_text += f"  - Number of RS paths attempted: {len(self.rs_path_summary)}\n"

        if self.coarse_path is not None:
            stats_text += f"\nCoarse Path:\n"
            stats_text += f"  - Number of path points: {len(self.coarse_path)}\n"
            stats_text += f"  - Path length: {np.sum(np.sqrt(np.diff(self.coarse_path['x'])**2 + np.diff(self.coarse_path['y'])**2)):.2f} m\n"
            stats_text += f"  - Average velocity: {self.coarse_path['v'].mean():.2f} m/s\n"
            stats_text += f"  - Maximum steering angle: {self.coarse_path['steer'].abs().max():.3f} rad\n"

        ax4.text(0.05, 0.95, stats_text, transform=ax4.transAxes,
                fontsize=10, verticalalignment='top', family='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.suptitle(f'Hybrid A* Search Results (Run ID: {self.run_id})', fontsize=14, fontweight='bold')

    def show(self):
        """Show all plots"""
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Hybrid A* Search Visualization Tool')
    parser.add_argument('--log-dir', type=str, default='csv_logs/hybrid_astar',
                       help='CSV log directory')
    parser.add_argument('--run-id', type=str, default='latest',
                       help='Run ID (use "latest" for the most recent)')
    parser.add_argument('--plot', type=str, default='comprehensive',
                       choices=['comprehensive', 'search_tree', 'cost', 'rs_paths', 'coarse_path', 'all'],
                       help='Type of plot to generate')

    args = parser.parse_args()

    try:
        vis = HybridAStarVisualizer(args.log_dir, args.run_id)

        if args.plot == 'comprehensive':
            vis.plot_comprehensive()
        elif args.plot == 'search_tree':
            vis.plot_search_tree()
        elif args.plot == 'cost':
            vis.plot_cost_distribution()
        elif args.plot == 'rs_paths':
            vis.plot_reed_shepp_paths()
        elif args.plot == 'coarse_path':
            vis.plot_coarse_path()
        elif args.plot == 'all':
            vis.plot_search_tree()
            vis.plot_cost_distribution()
            vis.plot_reed_shepp_paths()
            vis.plot_coarse_path()
            vis.plot_comprehensive()

        vis.show()

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
