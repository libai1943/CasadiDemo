#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complete Planning Pipeline Visualization Tool

Purpose:
- Visualize both Hybrid A* and NLP optimization results simultaneously
- Compare coarse path and optimized path
- Analyze complete planning pipeline
- Help debug planning failure issues

Usage:
    python3 visualize_complete_pipeline.py [--hastar-dir csv_logs/hybrid_astar] [--nlp-dir csv_logs/nlp_optimization]
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse
import glob
import os
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import PatchCollection


class CompletePipelineVisualizer:
    def __init__(self, hastar_dir="csv_logs/hybrid_astar",
                 nlp_dir="csv_logs/nlp_optimization",
                 hastar_run_id="latest",
                 nlp_run_id="latest"):
        """
        Initialize visualizer

        Args:
            hastar_dir: Hybrid A* log directory
            nlp_dir: NLP optimization log directory
            hastar_run_id: Hybrid A* run ID
            nlp_run_id: NLP optimization run ID
        """
        self.hastar_dir = Path(hastar_dir)
        self.nlp_dir = Path(nlp_dir)

        # Get run IDs
        if hastar_run_id == "latest":
            pattern = str(self.hastar_dir / "*_coarse_path.csv")
            files = glob.glob(pattern)
            if files:
                latest_file = max(files, key=os.path.getctime)
                self.hastar_run_id = Path(latest_file).stem.replace("_coarse_path", "")
            else:
                self.hastar_run_id = None
        else:
            self.hastar_run_id = hastar_run_id

        if nlp_run_id == "latest":
            pattern = str(self.nlp_dir / "*_final_result.csv")
            files = glob.glob(pattern)
            if files:
                latest_file = max(files, key=os.path.getctime)
                self.nlp_run_id = Path(latest_file).stem.replace("_final_result", "")
            else:
                self.nlp_run_id = None
        else:
            self.nlp_run_id = nlp_run_id

        print(f"Hybrid A* Run ID: {self.hastar_run_id}")
        print(f"NLP Optimization Run ID: {self.nlp_run_id}")

        # Load data
        self.load_data()

    def load_data(self):
        """Load all data"""
        # Hybrid A* data
        if self.hastar_run_id:
            coarse_file = self.hastar_dir / f"{self.hastar_run_id}_coarse_path.csv"
            if coarse_file.exists():
                self.coarse_path = pd.read_csv(coarse_file)
                print(f"Loaded coarse path: {len(self.coarse_path)} points")
            else:
                self.coarse_path = None
                print("Coarse path file not found")

            search_file = self.hastar_dir / f"{self.hastar_run_id}_search_nodes.csv"
            if search_file.exists():
                self.search_nodes = pd.read_csv(search_file)
                print(f"Loaded search nodes: {len(self.search_nodes)}")
            else:
                self.search_nodes = None
        else:
            self.coarse_path = None
            self.search_nodes = None

        # NLP optimization data
        if self.nlp_run_id:
            initial_file = self.nlp_dir / f"{self.nlp_run_id}_initial_guess.csv"
            if initial_file.exists():
                self.initial_guess = pd.read_csv(initial_file)
                print(f"Loaded initial guess: {len(self.initial_guess)} points")
            else:
                self.initial_guess = None
                print("Initial guess file not found")

            final_file = self.nlp_dir / f"{self.nlp_run_id}_final_result.csv"
            if final_file.exists():
                self.final_result = pd.read_csv(final_file)
                success = self.final_result['success'].iloc[0] if 'success' in self.final_result else 0
                print(f"Loaded final result: {len(self.final_result)} points, success={success}")
            else:
                self.final_result = None
                print("Final result file not found")

            iterations_file = self.nlp_dir / f"{self.nlp_run_id}_iterations.csv"
            if iterations_file.exists():
                self.iterations = pd.read_csv(iterations_file)
                print(f"Loaded iteration data: {self.iterations['iteration'].max()+1} iterations")
            else:
                self.iterations = None

            corridor_file = self.nlp_dir / f"{self.nlp_run_id}_corridor_constraints.csv"
            if corridor_file.exists():
                self.corridor = pd.read_csv(corridor_file)
                print(f"Loaded corridor constraints: {len(self.corridor)}")
            else:
                self.corridor = None
        else:
            self.initial_guess = None
            self.final_result = None
            self.iterations = None
            self.corridor = None

    def plot_complete_comparison(self):
        """Plot complete comparison view"""
        fig = plt.figure(figsize=(18, 12))
        gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

        # 1. Main trajectory comparison plot (occupies left 2x3)
        ax_main = fig.add_subplot(gs[:, 0:2])

        # Search tree (semi-transparent)
        if self.search_nodes is not None:
            nodes = self.search_nodes.head(3000)
            ax_main.scatter(nodes['x'], nodes['y'], c='gray', s=3, alpha=0.15, label='Search Nodes')

        # Coarse path
        if self.coarse_path is not None:
            ax_main.plot(self.coarse_path['x'], self.coarse_path['y'],
                        'b-', linewidth=3, alpha=0.6, label='Coarse Path (Hybrid A*)')

        # Initial guess
        if self.initial_guess is not None:
            ax_main.plot(self.initial_guess['x'], self.initial_guess['y'],
                        'g--', linewidth=2, alpha=0.7, label='Initial Guess')

        # Final optimized result
        if self.final_result is not None:
            sc = ax_main.scatter(self.final_result['x'], self.final_result['y'],
                               c=self.final_result['v'], cmap='coolwarm',
                               s=60, edgecolors='black', linewidths=0.5, label='Optimized Result', zorder=5)

            # Plot direction arrows
            step = max(1, len(self.final_result) // 20)
            for i in range(0, len(self.final_result), step):
                row = self.final_result.iloc[i]
                dx = 0.4 * np.cos(row['theta'])
                dy = 0.4 * np.sin(row['theta'])
                ax_main.arrow(row['x'], row['y'], dx, dy,
                            head_width=0.2, head_length=0.15,
                            fc='darkred', ec='darkred', alpha=0.7, zorder=6)

            plt.colorbar(sc, ax=ax_main, label='Velocity (m/s)', pad=0.01)

            # Start and end points
            ax_main.plot(self.final_result.iloc[0]['x'], self.final_result.iloc[0]['y'],
                        'go', markersize=15, label='Start', zorder=10)
            ax_main.plot(self.final_result.iloc[-1]['x'], self.final_result.iloc[-1]['y'],
                        'ro', markersize=15, label='End', zorder=10)

        # Corridor constraints (sampled display)
        if self.corridor is not None:
            step = max(1, len(self.corridor) // 50)
            for i in range(0, len(self.corridor), step):
                row = self.corridor.iloc[i]
                if row['is_front_disc'] == 0:  # Only show rear disc
                    rect = Rectangle((row['x_min'], row['y_min']),
                                   row['x_max'] - row['x_min'],
                                   row['y_max'] - row['y_min'],
                                   fill=True, alpha=0.05, color='green',
                                   edgecolor='green', linewidth=0.5)
                    ax_main.add_patch(rect)

        ax_main.set_xlabel('X (m)', fontsize=12)
        ax_main.set_ylabel('Y (m)', fontsize=12)
        ax_main.set_title('Complete Planning Pipeline Comparison', fontsize=14, fontweight='bold')
        ax_main.legend(loc='best', fontsize=10)
        ax_main.grid(True, alpha=0.3)
        ax_main.axis('equal')

        # 2. Velocity comparison
        ax_vel = fig.add_subplot(gs[0, 2])
        if self.coarse_path is not None:
            ax_vel.plot(self.coarse_path['index'], self.coarse_path['v'],
                       'b-', linewidth=2, alpha=0.7, label='Coarse Path')
        if self.final_result is not None:
            ax_vel.plot(self.final_result['index'], self.final_result['v'],
                       'r-', linewidth=2, label='Optimized Result')
        ax_vel.set_xlabel('Path Point Index')
        ax_vel.set_ylabel('Velocity (m/s)')
        ax_vel.set_title('Velocity Comparison')
        ax_vel.legend()
        ax_vel.grid(True, alpha=0.3)

        # 3. Acceleration comparison
        ax_acc = fig.add_subplot(gs[1, 2])
        if self.coarse_path is not None:
            ax_acc.plot(self.coarse_path['index'], self.coarse_path['a'],
                       'b-', linewidth=2, alpha=0.7, label='Coarse Path')
        if self.final_result is not None:
            ax_acc.plot(self.final_result['index'], self.final_result['a'],
                       'r-', linewidth=2, label='Optimized Result')
        ax_acc.set_xlabel('Path Point Index')
        ax_acc.set_ylabel('Acceleration (m/s²)')
        ax_acc.set_title('Acceleration Comparison')
        ax_acc.legend()
        ax_acc.grid(True, alpha=0.3)

        # 4. Steering angle comparison
        ax_steer = fig.add_subplot(gs[2, 2])
        if self.coarse_path is not None:
            ax_steer.plot(self.coarse_path['index'], self.coarse_path['steer'],
                         'b-', linewidth=2, alpha=0.7, label='Coarse Path')
        if self.final_result is not None:
            ax_steer.plot(self.final_result['index'], self.final_result['phi'],
                         'r-', linewidth=2, label='Optimized Result')
        ax_steer.set_xlabel('Path Point Index')
        ax_steer.set_ylabel('Steering Angle (rad)')
        ax_steer.set_title('Steering Angle Comparison')
        ax_steer.legend()
        ax_steer.grid(True, alpha=0.3)

        plt.suptitle('Parking Planning Complete Pipeline Analysis', fontsize=16, fontweight='bold', y=0.995)

    def plot_statistics(self):
        """Plot statistical information"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # 1. Path length comparison
        ax1 = axes[0, 0]
        lengths = []
        labels = []

        if self.coarse_path is not None:
            coarse_length = np.sum(np.sqrt(
                np.diff(self.coarse_path['x'])**2 + np.diff(self.coarse_path['y'])**2))
            lengths.append(coarse_length)
            labels.append('Coarse Path')

        if self.initial_guess is not None:
            initial_length = np.sum(np.sqrt(
                np.diff(self.initial_guess['x'])**2 + np.diff(self.initial_guess['y'])**2))
            lengths.append(initial_length)
            labels.append('Initial Guess')

        if self.final_result is not None:
            final_length = np.sum(np.sqrt(
                np.diff(self.final_result['x'])**2 + np.diff(self.final_result['y'])**2))
            lengths.append(final_length)
            labels.append('Optimized Result')

        ax1.bar(labels, lengths, color=['blue', 'green', 'red'])
        ax1.set_ylabel('Path Length (m)')
        ax1.set_title('Path Length Comparison')
        ax1.grid(True, alpha=0.3, axis='y')

        # 2. Execution time comparison
        ax2 = axes[0, 1]
        times = []
        time_labels = []

        if self.coarse_path is not None:
            times.append(len(self.coarse_path))
            time_labels.append('Coarse Points')

        if self.final_result is not None:
            times.append(len(self.final_result))
            time_labels.append('Optimized Points')

            if 't' in self.final_result.columns:
                total_time = self.final_result['t'].max()
                times.append(total_time)
                time_labels.append('Total Time (s)')

        ax2.bar(time_labels, times, color=['blue', 'red', 'orange'])
        ax2.set_ylabel('Value')
        ax2.set_title('Points and Time Comparison')
        ax2.grid(True, alpha=0.3, axis='y')

        # 3. Velocity statistics
        ax3 = axes[1, 0]
        if self.coarse_path is not None:
            ax3.hist(self.coarse_path['v'], bins=30, alpha=0.5, color='blue', label='Coarse Path')
        if self.final_result is not None:
            ax3.hist(self.final_result['v'], bins=30, alpha=0.5, color='red', label='Optimized Result')
        ax3.set_xlabel('Velocity (m/s)')
        ax3.set_ylabel('Frequency')
        ax3.set_title('Velocity Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # 4. Optimization convergence curve
        ax4 = axes[1, 1]
        if self.iterations is not None:
            infeas = self.iterations.groupby('iteration')['infeasibility'].first()
            ax4.plot(infeas.index, infeas.values, 'bo-', linewidth=2, markersize=8)
            ax4.set_xlabel('Iteration')
            ax4.set_ylabel('Infeasibility')
            ax4.set_title('NLP Optimization Convergence Curve')
            ax4.set_yscale('log')
            ax4.grid(True, alpha=0.3)
        else:
            ax4.text(0.5, 0.5, 'No optimization iteration data', ha='center', va='center',
                    transform=ax4.transAxes, fontsize=12)
            ax4.set_title('NLP Optimization Convergence Curve')

        plt.tight_layout()

    def plot_detailed_analysis(self):
        """Plot detailed analysis"""
        if self.final_result is None:
            print("No final result data")
            return

        fig, axes = plt.subplots(3, 2, figsize=(14, 12))

        t = self.final_result['t'] if 't' in self.final_result.columns else self.final_result['index']

        # Position
        axes[0, 0].plot(t, self.final_result['x'], 'r-', linewidth=2)
        axes[0, 0].set_xlabel('Time/Index')
        axes[0, 0].set_ylabel('X (m)')
        axes[0, 0].set_title('X Position')
        axes[0, 0].grid(True, alpha=0.3)

        axes[0, 1].plot(t, self.final_result['y'], 'b-', linewidth=2)
        axes[0, 1].set_xlabel('Time/Index')
        axes[0, 1].set_ylabel('Y (m)')
        axes[0, 1].set_title('Y Position')
        axes[0, 1].grid(True, alpha=0.3)

        # Heading angle and velocity
        axes[1, 0].plot(t, self.final_result['theta'], 'g-', linewidth=2)
        axes[1, 0].set_xlabel('Time/Index')
        axes[1, 0].set_ylabel('Heading Angle (rad)')
        axes[1, 0].set_title('Heading Angle')
        axes[1, 0].grid(True, alpha=0.3)

        axes[1, 1].plot(t, self.final_result['v'], 'purple', linewidth=2)
        axes[1, 1].set_xlabel('Time/Index')
        axes[1, 1].set_ylabel('Velocity (m/s)')
        axes[1, 1].set_title('Velocity')
        axes[1, 1].grid(True, alpha=0.3)

        # Control inputs
        axes[2, 0].plot(t, self.final_result['a'], 'orange', linewidth=2)
        axes[2, 0].set_xlabel('Time/Index')
        axes[2, 0].set_ylabel('Acceleration (m/s²)')
        axes[2, 0].set_title('Acceleration')
        axes[2, 0].grid(True, alpha=0.3)

        axes[2, 1].plot(t, self.final_result['omega'], 'brown', linewidth=2)
        axes[2, 1].set_xlabel('Time/Index')
        axes[2, 1].set_ylabel('Steering Rate (rad/s)')
        axes[2, 1].set_title('Steering Rate')
        axes[2, 1].grid(True, alpha=0.3)

        plt.tight_layout()

    def print_summary(self):
        """Print summary information"""
        print("\n" + "="*60)
        print("Planning Pipeline Summary")
        print("="*60)

        print("\n[Hybrid A* Search]")
        if self.search_nodes is not None:
            print(f"  Total search nodes: {len(self.search_nodes)}")
            print(f"  Forward nodes: {len(self.search_nodes[self.search_nodes['direction']==1])}")
            print(f"  Backward nodes: {len(self.search_nodes[self.search_nodes['direction']==0])}")
            print(f"  Average trajectory cost: {self.search_nodes['traj_cost'].mean():.2f}")

        if self.coarse_path is not None:
            coarse_length = np.sum(np.sqrt(
                np.diff(self.coarse_path['x'])**2 + np.diff(self.coarse_path['y'])**2))
            print(f"\n  Coarse path points: {len(self.coarse_path)}")
            print(f"  Coarse path length: {coarse_length:.2f} m")
            print(f"  Average velocity: {self.coarse_path['v'].mean():.2f} m/s")
            print(f"  Maximum steering angle: {self.coarse_path['steer'].abs().max():.3f} rad")

        print("\n[NLP Optimization]")
        if self.iterations is not None:
            num_iters = self.iterations['iteration'].max() + 1
            final_inf = self.iterations[self.iterations['iteration']==num_iters-1]['infeasibility'].iloc[0]
            print(f"  Number of iterations: {num_iters}")
            print(f"  Final infeasibility: {final_inf:.6e}")

        if self.final_result is not None:
            final_length = np.sum(np.sqrt(
                np.diff(self.final_result['x'])**2 + np.diff(self.final_result['y'])**2))
            success = self.final_result['success'].iloc[0] if 'success' in self.final_result else 0
            total_time = self.final_result['t'].max() if 't' in self.final_result else 0

            print(f"\n  Optimized result points: {len(self.final_result)}")
            print(f"  Optimized path length: {final_length:.2f} m")
            print(f"  Total execution time: {total_time:.2f} s")
            print(f"  Average velocity: {self.final_result['v'].mean():.2f} m/s")
            print(f"  Maximum acceleration: {self.final_result['a'].abs().max():.2f} m/s²")
            print(f"  Maximum steering angle: {self.final_result['phi'].abs().max():.3f} rad")
            print(f"  Optimization success: {'Yes' if success else 'No'}")

        print("\n" + "="*60 + "\n")

    def show(self):
        """Show all plots"""
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Complete Planning Pipeline Visualization Tool')
    parser.add_argument('--hastar-dir', type=str, default='csv_logs/hybrid_astar',
                       help='Hybrid A* log directory')
    parser.add_argument('--nlp-dir', type=str, default='csv_logs/nlp_optimization',
                       help='NLP optimization log directory')
    parser.add_argument('--hastar-run-id', type=str, default='latest',
                       help='Hybrid A* run ID')
    parser.add_argument('--nlp-run-id', type=str, default='latest',
                       help='NLP run ID')
    parser.add_argument('--plot', type=str, default='all',
                       choices=['comparison', 'statistics', 'analysis', 'all'],
                       help='Type of plot to generate')

    args = parser.parse_args()

    try:
        vis = CompletePipelineVisualizer(args.hastar_dir, args.nlp_dir,
                                        args.hastar_run_id, args.nlp_run_id)

        vis.print_summary()

        if args.plot == 'comparison':
            vis.plot_complete_comparison()
        elif args.plot == 'statistics':
            vis.plot_statistics()
        elif args.plot == 'analysis':
            vis.plot_detailed_analysis()
        elif args.plot == 'all':
            vis.plot_complete_comparison()
            vis.plot_statistics()
            vis.plot_detailed_analysis()

        vis.show()

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
