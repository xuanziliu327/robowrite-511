# ShanghaiTech University SI100B Project: RoboWriter
## Introduction
This is a README file for **ShanghaiTech University SI100B Project: RoboWriter**. This file explains the requirements and grading criteria of the project, including project implementations, project reports, and project presentations. Please read throughout this file and make sure you have a group member with Windows platform before starting the project.

## I. Project Implementation (45 points)
### Requirements
#### Basic
1. Robot Arm Model (2 points): The universal_robots_ur5e model in the newest version of code repo : https://github.com/MAYBERoboLab/SIST_SI100B_RoboWriter 
2. Start pose and Terminal pose (5 points): The robot arm can start at any pose but needs to stop at the joint configuration: $[0.0,-2.32,-1.38,-2.45,1.57,0.0]$ rad after finishing the writing.
3. Writing area (5 points): The writing area is a square at the plane z=0.1 m. This square is defined by its four vertices: $(0.5, 0.1, 0.1)$, $(0.5, 0.6, 0.1)$, $(-0.5, 0.6, 0.1)$, and $(-0.5, 0.1, 0.1)$ m.
4. Lifting the end-effector (5 points): The end-effector of the robot arm should lift between each stroke and each character.
5. Writing contents (10 points): The robot should write a student's name (can choose one in your group) in Chinese and PinYin, and the student's school ID in the writing area. Visualize the written scripts/strokes of the words in real time.
6. Interpolation Method (7 points): Use multiple different interpolation methods to mimic the real strokes.
7. Writing speed (3 points): The robot's writing speed should be set at a reasonable value so that the whole writing time (from the robot's start pose to the terminal pose) is less than 3 minutes.
8. Plot joint state curves (3 points): Plot all the joint states of the robot arm when wirtting one word, let it be your family name in Chinese. 
9. Demo video (5 points): Record a video to demonstrate the writing process, you can adjust the camera for a better view. The name of the video should be "Group Member Names + Final Project Demo" in MP4 format.

#### Bonus
1. Git (1 point): Use Git to manage your code, and you need to have at least 3 commits for each group member.
2. Writing arbitrary words (2 points): Given an arbitrary string, you need to write the words in the string with the robot arm. The string will only contain English letters, spaces, numbers, commas, and periods.
3. Writing on a sphere (3 points): Write words on an area of the inner surface of a sphere defined by: $(x-0)^2+(y-0.35)^2+(z-1.3)^2=0$ and $z\leq 0.1$.

### Grading Criteria
1. For basic requirements, if you will only earn the points from requirement 1 to $k (k=1,2,3,4,5,6,7,8)$ if you fail to finish requirement $k+1$. If you fail to finish multiple requirements, $k$ will be the smallest requirement index that was not completed.
2. For basic requirement 7, you will earn 3 points if you only use linear interpolation.
3. The bonus tasks can be completed in any combination


## II. Project Presentations (4 points)
### Requirements
#### I. PPT Guidelines.
1. Language: English
2. Front Page Content: Title, Names of Group Members, and Student IDs.
3. Core Content: Must include clear project execution results --Plots and Videos.
#### II.Time Allocation (5 Minutes Total)
1. 4 Minutes: PPT presentation 
2. 1 Minute: Q&A session.

### Grading Criteria
1. Time Management [15%]: Timing must be within a ±1 minute margin. 
2. Visual Clarity [10%]: Slides should have a clear layout that is professional and easy to read.
3. Logical Structure [60%]: The presentation should include Methodology, Results, Limitations, and Future Work/Possible Improvements.
4. Q&A Session [15%]: Ability to provide clear answers to questions.


## III. Project Reports (6 points)
### Requirements
1.  Project Overview: Briefly introduce the research background, research purpose, application scenarios and overall work content of the project.
2.  System Design: Describe the overall architecture of the system, module divisions, key technical solutions, and the relationship between each module.
3.  Implementation: Describe in detail the specific implementation process of the system or project, including the technologies, algorithms, tools and implementation steps used.
4.  Testing and Results: Introduce the testing method and test environment, and display the system operation results and performance analysis.
5.  Discussion and Improvement: Analyze the significance of the experimental results, existing problems in the system, and propose possible improvement directions.
6.  Conclusion: Summarize the work and results of the overall project and briefly look forward to future research or applications.
 
### Grading creteria
1.  Completeness(25%): Evaluate whether the report contains all required chapters, whether the content is complete, whether the structure is clear, and whether key parts are not missing.
2.  Technical Explanation(30%): Evaluate whether the explanations of system design, technical solutions and implementation processes are accurate and clear, and reflect understanding of relevant technologies.
3.  Results Analysis(25%): Evaluate whether the analysis of experimental results or system output is reasonable and whether it can be effectively explained in combination with data or phenomena.
4.  Reflection and Improvement(20%): Evaluate the depth of reflection on project shortcomings and whether the proposed improvements are feasible and meaningful.
