# Robot-Memory-Game
Analyzing How Different Robot Signals Aid in Human Memory through an Interactive Memory Game

#### Link to Code:
https://github.com/aashiperun/Robot-Memory-Game/blob/main/Robot_Memory_Game_m1.py

https://github.com/aashiperun/Robot-Memory-Game/blob/main/Robot_Memory_Game_m2.py

### ABSTRACT
A key aspect in communication is capturing a person's attention for a long period of time. In HRI, it is challenging to design a robot that not only captures a human's attention but also holds it over time. With use cases where humans are learning from robots, for example, robot tutors in a classroom, it becomes increasingly important to test the human’s memory when they are taught by a robot as compared to a human. In this paper we explore different ways to solve this attention problem by creating an interactive memory game. The memory game had two rounds that took place on the same grid we created on the floor. In the first round, the robot traced a pattern on the grid, and in the second round the human was asked to retrace that pattern on the grid using a controller to test their memory. We evaluated the human’s memory based on the number of points they got right while retracing the pattern created by the robot. The goal of this experiment is to understand if a sound signal created by a robot is more memorable to humans. The results of this study could positively impact HRI by helping bring confidence to researchers who would want to use robots to gain attention and relay important information to humans.

### 1. INTRODUCTION
As human-robot interaction becomes more prevalent in work environments, it is important to understand the mechanisms of communication and signaling. We want to examine the memory capacity and attentional abilities of humans when asked to replicate a robot’s actions. Capturing the attention of the user may be crucial for a robot to successfully aid in a task related to search and rescue, healthcare, or military engagements. Another consideration in HRI research is the memory capacity of humans, and the ability to mentally track a robot’s movements. This may be important for tasks that require counting or keeping a record of certain locations. Both memory capacity and attention of the user must be examined in order to better produce robot assistants in the field. 

<img width="343" alt="image" src="https://user-images.githubusercontent.com/66789469/218501705-980021c0-403c-4746-980a-9df7d65e80ed.png">
Our research question is which robot signals produce the highest memory retention from users: sound signal, or no sound signal. In order to answer this question, we designed an experiment in which a participant is completing a memory game with a robot (Figure 1). The robot moves around a map of different color stickers and gives either sound signal or no signal to the participant at each location. Once it is finished, we ask the participant to retrace the path of the robot and measure the accuracy of the path. We will then compare accuracy across the different signal conditions to determine the signal mechanisms that produce the best results. We hypothesize that sound signals will have the highest memory retention rate because it adds another sensory element other than visual. The added sensory element may increase attention and memorability of the robot’s actions. 

Our research question is which robot signals produce the highest memory retention from users: sound signal, or no sound signal. In order to answer this question, we designed an experiment in which a participant is completing a memory game with a robot (Figure 1). The robot moves around a map of different color stickers and gives either sound signal or no signal to the participant at each location. Once it is finished, we ask the participant to retrace the path of the robot and measure the accuracy of the path. We will then compare accuracy across the different signal conditions to determine the signal mechanisms that produce the best results. We hypothesize that sound signals will have the highest memory retention rate because it adds another sensory element other than visual. The added sensory element may increase attention and memorability of the robot’s actions. 

### 2. RELATED WORK
There have been several studies relating to human-robot interaction, memory, and signal processing. Attention is important for human-robot collaboration, as human perception of robot action and intention relies on visual or auditory signals. Memory ability can be a useful indication of which signals are more engaging, and therefore receive more attention from users. In our study, we aim to identify the signals that improve attention and memory in human-robot interactive tasks. Prior research has shown that visual attention improves task performance (Lindsay, 2018). In this paper, we will further explore the concept of attention in HRI in relation to sound instead of vision. 

In addition to attention and signaling, the use of memory has been explored in human-robot social interactions. One study explores social facilitation in human-robot interaction by implementing a memory game (Cruz-Maya, et al., 2015). The paper focuses on emotion and memory as a way to build relationships between humans and robots over time. Though our experiment explores short term memory, the concept of using memory in a game-like setting is applicable to our topic of interest. The broader implication of this research could impact how we design robots to establish a memorable and effective signal system. 

In the field of HRI, there has also been work done focusing on the effects of different nonverbal communication methods and how they affected cognitive framing, emotion recognition and response, behavioral response and task performance in people. In a 2019 paper, researchers investigated the connection between different non-verbal communications focusing on how they affected human outcomes. The results suggest that the use of robot eye gaze, gestures or facial expressions all decreased the time it took to complete tasks (Saunderson, 2019). This implies that the nonverbal communication in our experiment should result in better memory than the control condition, with no sound signal. The paper also mentions that more research needs to be done on the effect of different behavior types and how the combination would affect human performance. Our experiment aims to explore the topic further, focusing on the sound signaling in human-robot interactions. 

### 3. METHODS
##### 3.1 Participants
We recruited 10 Cornell Tech students from different programs to participate in the Memory Game. The mean of ages of our participants was 24, with a standard deviation of 2. We were able to recruit five female participants and five male participants. We used convenience sampling to recruit participants.

##### 3.2 Design Robot Behavior 
Our study setup involved a grid of colored dots on the floor as shown in Figure 1. We went through multiple different strategies to have the robot create a path autonomously. We created a state machine to move the robot to the next point and make a sound to alert the user after each move (Figure 2).

<img width="283" alt="image" src="https://user-images.githubusercontent.com/66789469/218502056-5c372081-81d2-4e0a-84b9-990b689bb3e7.png">
Robot Architecture for Round 1: 
The first state was the start state where the robot’s initial position is at the docking station. In the second state, the robot moves to the start line which is its first goal position and then alerts the user that the game is going to start. The robot then creates a pattern using the next six goal positions, with the chosen expression (sound or no sound) in state three. When the pattern is complete, the robot moves onto state four where it goes back to the start line, which is the first goal position again and alerts the user that it is their turn to play. Then, the robot concludes by coming to a stop in state five. Our initial goal was to design the robot to move to different points by generating waypoints on a map produced using Simultaneous Localization and Mapping (SLAM) (Figure 3). Unfortunately, the robot traced a different pattern each time which made it unreliable for human testing.  

<img width="293" alt="image" src="https://user-images.githubusercontent.com/66789469/218502791-761b1d2f-c81e-4631-b43e-b14332a80edf.png">
For our second approach, we used the robot to publish to the command velocity (\cmd_vel) topic to create a pattern using the move_robot function (Figure 4). We finally used the second approach as it was more reliable to create a path on the map.

<img width="311" alt="image" src="https://user-images.githubusercontent.com/66789469/218502934-947c61b2-d03c-4dc3-b9b5-b9125e8be747.png">
The memory game was played in two rounds. In Round 1, the robot moved from the docking station to the start line and then proceeded to create a pattern on the grid. We used Reactive Architecture for Round 1. The study setup in round 1 is shown in Figure 5. In Round 2, a person uses the controller to retrace the pattern created by the robot in round 1 (Figure 5). To set up the controller we used the command “systemctl start hciuart”. We then paired the turtlebot with the controller using bluetooth through the MAC address.

<img width="321" alt="image" src="https://user-images.githubusercontent.com/66789469/218503152-9d3daada-62fa-4a1c-9bec-8cefeb26a6b9.png">

##### 3.3 Study Design
We will conduct between-subject studies, since the memory capacity of participants over a longer period of time may change. This is the best design for our experiment, because we are able to measure the memory capabilities of participants without the threat of order influencing the results. The study is human-focused, as we are measuring the participants ability to remember the actions of the robot. Since we are undertaking a between subjects study, we do not account for counterbalancing. Below is the factorial design for the experiment (2 x 1) as shown in Table 1.

<img width="324" alt="image" src="https://user-images.githubusercontent.com/66789469/218503514-cba04a86-b5ae-428b-8c6f-02ecab241d81.png">
Number of treatment groups:
GB = IV x DV = 2 x 1 = 2 groups
Number of study sessions:
NB = GB x P = 2 x 5 = 10 study sessions
Total in-study time:
TB = NB x S = 10 x 15 mins = 150 mins (>2 hours)

For our study, the independent variables are the robot expressions - sound or no sound (control). The dependent variable is the True Positive Rate which is the number of points the participant correctly retraces out of the total number of points on the pattern.

##### 3.4 Study Task
Once the participant enters the room, we read out the following script to explain the purpose of our study.	
	
“Hello, I am _____. My colleagues ____ and I are exploring how effective people’s memory is when they are given information by robots. You will first watch a robot create a pattern on the grid. Then you will try to recreate the pattern by moving the robot using the controller. Hope you have fun playing our memory game!” 


Next, the participants were asked to fill out an online consent form which we created using google forms. The consent form asked the participants their name, age, gender identity and if they were willing to take part in our study.
In the first round, the participant watches the robot trace a path and try to memorize the pattern. In the second round, the participant uses the controller to retrace the robot's path from memory (Figure 1).  For the actual study, we used WoZ to control the robot using the Keyboard Teleop function since the robot wasn’t moving to all the points accurately. We had one person on our team recording the points to which the participant was moving the robot.

##### 3.6 Data Collection
During the study, we recorded a video of the interaction of the user and the robot.  Also, during the game, one researcher records the path the participants traced on paper for evaluation. The combination of the video and paper record helped to check for discrepancies and validate the results obtained. 

### 4. EVALUATION

##### 4.1  Experimental Metrics
This was a human-focused study that observed the memory retention of the participants. Each participant has one attempt to complete the pattern. The main metric was the number of points on the pattern that were correctly traced. 
We also used the formula below to calculate the true positive rate (TPR):

<img width="292" alt="image" src="https://user-images.githubusercontent.com/66789469/218504053-a545c244-dae1-4535-8510-a5ad7112c18e.png">

These metrics help to compare the effectiveness of different robot signals (sound and no sound)  in improving memory retention. 

##### 4.2  Data Analysis
We collected and analyzed data from two mediums: video of the participant interacting with the robot, and path tracing on paper by a designated researcher. A sample video recorded is attached in appendix A.1.  Also, below in Figure 7 are some sample paper records traced from the sound signal condition and the no sound condition. The number of correct and incorrect points were collected into an excel document and graphs produced per condition.

<img width="333" alt="image" src="https://user-images.githubusercontent.com/66789469/218504182-4e8a4228-b1ab-4e9b-ba18-7ed5b3fb23f3.png">

### 5. RESULTS

There was a numerical difference between the TPR of participants in the sound condition compared to the control. However, these results are not statistically significant, so we are not able to make a conclusion supporting our hypothesis. In the sound condition, all 5 participants successfully replicated the path from memory without any errors. The average TPR was 100 with a standard deviation of 0. In the control group, 1 out of the 5 participants did not correctly retrace the path. The average TPR was 88.4, with a standard deviation of 25.9 (Table 2 and Figure 8).  

These results did surprise us because we weren’t expecting so many people to successfully play the game without any errors. If we were to redo the experiment, we would make it more difficult by adding more for the participant to remember or getting rid of the colors. The colors seemed to help participants remember the path more easily. 

Since our experiment was human-focused, our results reflect the performance of participants, not the turtlebot. We went through trial and error with different technical approaches, and found the wizard of Oz approach to be most reliable. We came across quite a lot of technical issues. The battery life of the turtlebot was very poor and it died for one of our participants. Plus, the bluetooth was not possible for one of the turtlebots, so we ended up changing the turtlebot used for the study.

<img width="305" alt="image" src="https://user-images.githubusercontent.com/66789469/218504382-af97417e-ed74-42af-b478-d177f0f68125.png">

### 6. DISCUSSION AND FUTURE WORK
Overall, our experiment indicates that more research is needed to understand how robot signaling plays a role in human memory. Our results were not significantly different, as only one person made a mistake in the control. Since the difference between the Sound and No Sound condition was very low, there may be issues with our experimental design and set up. In the future, it would be beneficial to observe more participants so that there may be strong statistical patterns observed. 

Additionally, we noticed a Ceiling Effect in our experiment. This means that a high portion of subjects reached the maximum score possible. To avoid this in the future, we would need to make the path more complex, so participants aren’t able to easily replicate the path. Additionally, we should increase the time between when the participant sees the path and when they are asked to replicate it. After more time, participants may not remember the path as well. 

In terms of the robot design, future work is needed to explore other types of signaling, like light and gesture signals. Understanding the role of sound is helpful when designing effective robot signals, and exploring this further would be beneficial for the field of HRI. 

#### 7. References:
1. Cruz-Maya, Arturo, et al. “Social Facilitation in a Game-like Human-Robot Interaction Using Synthesized Emotions and Episodic Memory.” Social Robotics, 2015, pp. 164–173.,https://doi.org/10.1007/978-3-319-25554-5_17. 
2. Huang, Chien-Ming, and Andrea L. Thomaz. “Effects of Responding to, Initiating and Ensuring Joint Attention in Human-Robot Interaction.” 2011 RO-MAN, 2011, https://doi.org/10.1109/roman.2011.6005230.
3. Lindsay, Grace W, and Kenneth D Miller. “How Biological Attention Mechanisms Improve Task Performance in a Large-Scale Visual System Model.” ELife, vol. 7, 2018, https://doi.org/10.7554/elife.38105. 
4. Saunderson, S., Nejat, G. How Robots Influence Humans: A Survey of Nonverbal Communication in Social Human-Robot Interaction. Int J of Sco Robotics 11, 575-608 (2019). https://doi.org/10.1007/s12369-019-00523-0




