from Agent import Agent
import random

agent = Agent()

for n in range(10):
    agent.iterate()
    agent.evaluate()
    print()
    print()

