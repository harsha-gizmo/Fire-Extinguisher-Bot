# Fire-Extinguisher-Bot
An AI Fire Extinguisher Bot which is capable of skillfully navigating in a grid layout while avoiding the threat of spreading fire, to activate the fire suppression system.

# Abstract:

In this project, we are addressing the challenges faced by an autonomous robotic entity aboard the
deep space vessel Archaeopteryx. The robot is tasked with responding to Fire emergencies while the crew
is in hibernation. The vessel's layout is meticulously structured on a 40x40 grid, with cells designated
as 'blocked' or 'open' through a systematic opening process, ensuring that approximately 60% of the
grid is accessible. The robot's mission is to navigate to a button that activates the Fire suppression
system, while skillfully avoiding the threat of spreading Fire, which can start randomly in an open cell.
We have developed four robust strategies for the robot, including a static pathfinding approach that
disregards Fire spread, a dynamic replanning method that accounts for current Fire locations, a cautious
strategy that avoids both Fire and its immediate vicinity, and a custom-designed robot incorporating
innovative decision-making algorithms. We are rigorously evaluating each robot's performance across
multiple simulations with varying Fire spread probabilities, with the firm goal of analyzing success rates
and identifying failure causes. Our objective is to explore different strategies for addressing challenges in
various environments, improving our understanding of optimal methods and designing algorithms with
high success rates.
