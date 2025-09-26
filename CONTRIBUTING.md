# In ~/aerosky_ws/CONTRIBUTING.md

# Contributing to AeroSky OS

We're excited you want to build an app for AeroSky OS! Our goal is to create a rich ecosystem of drone applications.

## How to Add Your App

1.  **Fork this Repository:** Start by forking the `aerosky_ws` repository on GitHub.

2.  **Create Your Package:** Navigate to the `src` directory and create a new ROS 2 package for your app.
    ```bash
    cd aerosky_ws/src
    ros2 pkg create --build-type ament_python your_app_name
    ```

3.  **Write Your Code:** Add your Python logic inside the inner `your_app_name` directory. Your main file should be a ROS 2 Node that subscribes to and publishes PX4 messages.

4.  **Make it Executable:** Edit the `setup.py` file in your package to add an entry point. This allows `ros2 run` to find your app.
    ```python
    entry_points={
        'console_scripts': [
            'runner_name = your_app_name.your_main_file:main',
        ],
    },
    ```

5.  **Build and Test:** From the root of the workspace (`~/aerosky_ws`), run `colcon build` and test your app with `ros2 run your_app_name runner_name`.

6.  **Submit a Pull Request:** Once your app is working, commit the code to your fork and open a Pull Request to the main AeroSky OS repository. We'll review it and add it to the ecosystem!
