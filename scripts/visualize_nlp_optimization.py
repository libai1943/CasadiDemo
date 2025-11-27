#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NLP Optimization Process Visualization Tool

Purpose:
- Visualize NLP optimization iteration process
- Compare initial guess and optimization results
- Display corridor constraints
- Analyze optimization failure reasons

Usage:
    python3 visualize_nlp_optimization.py [--log-dir csv_logs/nlp_optimization] [--run-id latest]
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from pathlib import Path
import argparse
import os
import glob
from matplotlib.patches import Rectangle


class NLPOptimizationVisualizer:
    def __init__(self, log_dir="csv_logs/nlp_optimization", run_id="latest"):
        """
        Initialize visualizer

        Args:
            log_dir: CSV log directory
            run_id: Run ID, "latest" means use the most recent one
        """
        self.log_dir = Path(log_dir)

        if run_id == "latest":
            # Find the latest run ID
            pattern = str(self.log_dir / "*_final_result.csv")
            files = glob.glob(pattern)
            if not files:
                raise ValueError(f"No log files found in {log_dir}")
            latest_file = max(files, key=os.path.getctime)
            self.run_id = Path(latest_file).stem.replace("_final_result", "")
        else:
            self.run_id = run_id

        print(f"Using run ID: {self.run_id}")

        # Load data
        self.load_data()

    def load_data(self):
        """Load all CSV data"""
        try:
            # Initial guess
            initial_file = self.log_dir / f"{self.run_id}_initial_guess.csv"
            if initial_file.exists():
                self.initial_guess = pd.read_csv(initial_file)
                print(f"Loaded initial guess: {len(self.initial_guess)} points")
            else:
                self.initial_guess = None
                print(f"Initial guess file not found: {initial_file}")

            # Iteration results
            iterations_file = self.log_dir / f"{self.run_id}_iterations.csv"
            if iterations_file.exists():
                self.iterations = pd.read_csv(iterations_file)
                self.num_iterations = self.iterations['iteration'].max() + 1
                print(f"Loaded iteration results: {self.num_iterations} iterations")
            else:
                self.iterations = None
                self.num_iterations = 0
                print(f"Iterations file not found: {iterations_file}")

            # Final result
            final_file = self.log_dir / f"{self.run_id}_final_result.csv"
            if final_file.exists():
                self.final_result = pd.read_csv(final_file)
                success = self.final_result['success'].iloc[0] if 'success' in self.final_result else 0
                print(f"Loaded final result: {len(self.final_result)} points, success={success}")
            else:
                self.final_result = None
                print(f"Final result file not found: {final_file}")

            # Corridor constraints
            corridor_file = self.log_dir / f"{self.run_id}_corridor_constraints.csv"
            if corridor_file.exists():
                self.corridor = pd.read_csv(corridor_file)
                print(f"Loaded corridor constraints: {len(self.corridor)} constraints")
            else:
                self.corridor = None
                print(f"Corridor constraints file not found: {corridor_file}")

        except Exception as e:
            print(f"Error loading data: {e}")
            raise

    def plot_initial_guess(self):
        """Plot initial guess"""
        if self.initial_guess is None:
            print("No initial guess data")
            return

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # Trajectory plot
        ax1 = axes[0, 0]
        sc = ax1.scatter(self.initial_guess['x'], self.initial_guess['y'],
                        c=self.initial_guess['v'], cmap='coolwarm', s=50)
        ax1.plot(self.initial_guess.iloc[0]['x'], self.initial_guess.iloc[0]['y'],
                'go', markersize=12, label='Start')
        ax1.plot(self.initial_guess.iloc[-1]['x'], self.initial_guess.iloc[-1]['y'],
                'ro', markersize=12, label='End')
        plt.colorbar(sc, ax=ax1, label='Velocity (m/s)')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Initial Guess Trajectory')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        # Velocity curve
        ax2 = axes[0, 1]
        ax2.plot(self.initial_guess['t'], self.initial_guess['v'], 'b-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity Curve')
        ax2.grid(True, alpha=0.3)

        # Steering angle curve
        ax3 = axes[1, 0]
        ax3.plot(self.initial_guess['t'], self.initial_guess['phi'], 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Steering Angle (rad)')
        ax3.set_title('Steering Angle Curve')
        ax3.grid(True, alpha=0.3)

        # Control inputs
        ax4 = axes[1, 1]
        ax4.plot(self.initial_guess['t'], self.initial_guess['a'], 'r-', linewidth=2, label='Acceleration')
        ax4.plot(self.initial_guess['t'], self.initial_guess['omega'], 'b-', linewidth=2, label='Steering Rate')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Control Input')
        ax4.set_title('Control Inputs')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()

    def plot_iteration(self, iteration):
        """
        Plot results of a specific iteration

        Args:
            iteration: Iteration number
        """
        if self.iterations is None:
            print("No iteration data")
            return

        iter_data = self.iterations[self.iterations['iteration'] == iteration]
        if len(iter_data) == 0:
            print(f"Iteration {iteration} has no data")
            return

        infeasibility = iter_data['infeasibility'].iloc[0]

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # Trajectory plot
        ax1 = axes[0, 0]
        sc = ax1.scatter(iter_data['x'], iter_data['y'],
                        c=iter_data['v'], cmap='coolwarm', s=50)
        ax1.plot(iter_data.iloc[0]['x'], iter_data.iloc[0]['y'],
                'go', markersize=12, label='Start')
        ax1.plot(iter_data.iloc[-1]['x'], iter_data.iloc[-1]['y'],
                'ro', markersize=12, label='End')

        # Plot direction arrows
        step = max(1, len(iter_data) // 15)
        for i in range(0, len(iter_data), step):
            row = iter_data.iloc[i]
            dx = 0.3 * np.cos(row['theta'])
            dy = 0.3 * np.sin(row['theta'])
            ax1.arrow(row['x'], row['y'], dx, dy,
                     head_width=0.15, head_length=0.1, fc='black', ec='black', alpha=0.5)

        plt.colorbar(sc, ax=ax1, label='Velocity (m/s)')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title(f'Iteration {iteration} Trajectory (infeasibility={infeasibility:.6f})')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        # State variables
        ax2 = axes[0, 1]
        ax2.plot(iter_data['t'], iter_data['v'], 'b-', label='Velocity')
        ax2.plot(iter_data['t'], iter_data['phi'], 'g-', label='Steering Angle')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('State')
        ax2.set_title('State Variables')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Control inputs
        ax3 = axes[1, 0]
        ax3.plot(iter_data['t'], iter_data['a'], 'r-', label='Acceleration')
        ax3.plot(iter_data['t'], iter_data['omega'], 'b-', label='Steering Rate')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Control Input')
        ax3.set_title('Control Inputs')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Heading angle
        ax4 = axes[1, 1]
        ax4.plot(iter_data['t'], iter_data['theta'], 'purple', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Heading Angle (rad)')
        ax4.set_title('Heading Angle')
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()

    def plot_infeasibility_evolution(self):
        """Plot infeasibility evolution"""
        if self.iterations is None:
            print("No iteration data")
            return

        # Get infeasibility for each iteration
        infeas_data = self.iterations.groupby('iteration')['infeasibility'].first()

        plt.figure(figsize=(12, 6))
        plt.plot(infeas_data.index, infeas_data.values, 'bo-', linewidth=2, markersize=8)
        plt.xlabel('Iteration')
        plt.ylabel('Infeasibility')
        plt.title('Optimization Convergence Curve')
        plt.yscale('log')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()

        # Print convergence info
        print("\nConvergence Info:")
        for i, inf in infeas_data.items():
            print(f"  Iteration {i}: Infeasibility = {inf:.6e}")

    def plot_comparison(self):
        """Compare initial guess and final result"""
        if self.initial_guess is None or self.final_result is None:
            print("Missing initial guess or final result data")
            return

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # Trajectory comparison
        ax1 = axes[0, 0]
        ax1.plot(self.initial_guess['x'], self.initial_guess['y'],
                'b--', linewidth=2, alpha=0.7, label='Initial Guess')
        ax1.plot(self.final_result['x'], self.final_result['y'],
                'r-', linewidth=2, label='Optimized Result')
        ax1.plot(self.initial_guess.iloc[0]['x'], self.initial_guess.iloc[0]['y'],
                'go', markersize=12, label='Start')
        ax1.plot(self.final_result.iloc[-1]['x'], self.final_result.iloc[-1]['y'],
                'ro', markersize=12, label='End')

        # Plot corridor constraints (if available)
        if self.corridor is not None:
            # Only display some constraints to avoid overcrowding
            step = max(1, len(self.corridor) // 40)
            for i in range(0, len(self.corridor), step):
                row = self.corridor.iloc[i]
                if row['is_front_disc'] == 1:  # Only show front or rear disc
                    rect = Rectangle((row['x_min'], row['y_min']),
                                    row['x_max'] - row['x_min'],
                                    row['y_max'] - row['y_min'],
                                    fill=True, alpha=0.1, color='green', edgecolor='green', linewidth=0.5)
                    ax1.add_patch(rect)

        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Trajectory Comparison')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        # Velocity comparison
        ax2 = axes[0, 1]
        ax2.plot(self.initial_guess['t'], self.initial_guess['v'],
                'b--', linewidth=2, alpha=0.7, label='Initial Guess')
        ax2.plot(self.final_result['t'], self.final_result['v'],
                'r-', linewidth=2, label='Optimized Result')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity Comparison')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Steering angle comparison
        ax3 = axes[1, 0]
        ax3.plot(self.initial_guess['t'], self.initial_guess['phi'],
                'b--', linewidth=2, alpha=0.7, label='Initial Guess')
        ax3.plot(self.final_result['t'], self.final_result['phi'],
                'r-', linewidth=2, label='Optimized Result')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Steering Angle (rad)')
        ax3.set_title('Steering Angle Comparison')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Acceleration comparison
        ax4 = axes[1, 1]
        ax4.plot(self.initial_guess['t'], self.initial_guess['a'],
                'b--', linewidth=2, alpha=0.7, label='Initial Guess')
        ax4.plot(self.final_result['t'], self.final_result['a'],
                'r-', linewidth=2, label='Optimized Result')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Acceleration (m/s²)')
        ax4.set_title('Acceleration Comparison')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()

    def create_animation(self, save_path=None):
        """
        Create animation of optimization process

        Args:
            save_path: Save path (if None, don't save)
        """
        if self.iterations is None or self.num_iterations == 0:
            print("No iteration data")
            return

        fig, ax = plt.subplots(figsize=(12, 10))

        def animate(iteration):
            ax.clear()

            iter_data = self.iterations[self.iterations['iteration'] == iteration]
            if len(iter_data) == 0:
                return

            infeasibility = iter_data['infeasibility'].iloc[0]

            # Plot trajectory
            sc = ax.scatter(iter_data['x'], iter_data['y'],
                          c=iter_data['v'], cmap='coolwarm', s=50, vmin=-5, vmax=5)

            # Plot initial guess (transparent)
            if self.initial_guess is not None:
                ax.plot(self.initial_guess['x'], self.initial_guess['y'],
                       'b--', linewidth=1, alpha=0.3, label='Initial Guess')

            # Plot start and end points
            ax.plot(iter_data.iloc[0]['x'], iter_data.iloc[0]['y'],
                   'go', markersize=12, label='Start')
            ax.plot(iter_data.iloc[-1]['x'], iter_data.iloc[-1]['y'],
                   'ro', markersize=12, label='End')

            # Plot direction arrows
            step = max(1, len(iter_data) // 15)
            for i in range(0, len(iter_data), step):
                row = iter_data.iloc[i]
                dx = 0.3 * np.cos(row['theta'])
                dy = 0.3 * np.sin(row['theta'])
                ax.arrow(row['x'], row['y'], dx, dy,
                        head_width=0.15, head_length=0.1, fc='black', ec='black', alpha=0.5)

            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title(f'NLP Optimization Process - Iteration {iteration}/{self.num_iterations-1}\nInfeasibility={infeasibility:.6e}')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.axis('equal')

            plt.colorbar(sc, ax=ax, label='Velocity (m/s)')

        anim = animation.FuncAnimation(fig, animate, frames=self.num_iterations,
                                      interval=500, repeat=True)

        if save_path is not None:
            print(f"Saving animation to {save_path} ...")
            anim.save(save_path, writer='pillow', fps=2)
            print("Animation saved")

        return anim

    def plot_corridor_constraints(self):
        """Plot corridor constraints"""
        if self.corridor is None:
            print("No corridor constraint data")
            return

        plt.figure(figsize=(12, 10))

        # Separate front and rear disc constraints
        front = self.corridor[self.corridor['is_front_disc'] == 1]
        rear = self.corridor[self.corridor['is_front_disc'] == 0]

        # Plot front disc constraints
        for i in range(len(front)):
            row = front.iloc[i]
            rect = Rectangle((row['x_min'], row['y_min']),
                           row['x_max'] - row['x_min'],
                           row['y_max'] - row['y_min'],
                           fill=True, alpha=0.2, color='blue', edgecolor='blue', linewidth=1)
            plt.gca().add_patch(rect)

        # Plot rear disc constraints
        for i in range(len(rear)):
            row = rear.iloc[i]
            rect = Rectangle((row['x_min'], row['y_min']),
                           row['x_max'] - row['x_min'],
                           row['y_max'] - row['y_min'],
                           fill=True, alpha=0.2, color='red', edgecolor='red', linewidth=1)
            plt.gca().add_patch(rect)

        # Plot final trajectory
        if self.final_result is not None:
            plt.plot(self.final_result['x'], self.final_result['y'],
                    'k-', linewidth=3, label='Final Trajectory')

        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Corridor Constraints')
        plt.legend(['Front Disc', 'Rear Disc', 'Final Trajectory'])
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()

    def show(self):
        """Show all plots"""
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='NLP Optimization Visualization Tool')
    parser.add_argument('--log-dir', type=str, default='csv_logs/nlp_optimization',
                       help='CSV log directory')
    parser.add_argument('--run-id', type=str, default='latest',
                       help='Run ID (use "latest" for the most recent)')
    parser.add_argument('--plot', type=str, default='comparison',
                       choices=['initial', 'iteration', 'infeasibility', 'comparison',
                               'corridor', 'animation', 'all'],
                       help='Type of plot to generate')
    parser.add_argument('--iteration', type=int, default=0,
                       help='Iteration number to display (only for --plot iteration)')
    parser.add_argument('--save-animation', type=str, default=None,
                       help='Path to save animation (only for --plot animation)')

    args = parser.parse_args()

    try:
        vis = NLPOptimizationVisualizer(args.log_dir, args.run_id)

        if args.plot == 'initial':
            vis.plot_initial_guess()
        elif args.plot == 'iteration':
            vis.plot_iteration(args.iteration)
        elif args.plot == 'infeasibility':
            vis.plot_infeasibility_evolution()
        elif args.plot == 'comparison':
            vis.plot_comparison()
        elif args.plot == 'corridor':
            vis.plot_corridor_constraints()
        elif args.plot == 'animation':
            anim = vis.create_animation(args.save_animation)
        elif args.plot == 'all':
            vis.plot_initial_guess()
            vis.plot_infeasibility_evolution()
            vis.plot_comparison()
            vis.plot_corridor_constraints()
            if vis.num_iterations > 0:
                for i in range(min(3, vis.num_iterations)):
                    vis.plot_iteration(i)

        vis.show()

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
