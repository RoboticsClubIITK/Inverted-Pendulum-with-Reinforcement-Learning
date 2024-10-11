from environment import InvertedPendulumEnv
import numpy as np
from agent_sac import Agent
from plotting import plot_learning_curve
import time
import math


if __name__ == '__main__':
    
    env = InvertedPendulumEnv()
    state = env.reset()
    agent = Agent(input_dims=[5], env=env,
            n_actions=1)
    n_games = 400
    filename = 'inverted_pendulum.png'
    
    

    figure_file = filename

    best_score = env.reward_range[0]
    score_history = []
    load_checkpoint = True
    agent.load_models()
    env.render()
    if load_checkpoint:
        agent.load_models() 
        env.render()
    
    for i in range(n_games):
        env._get_state()
        observation = env.reset()
        observation = [observation[0]/5000 , observation[2]/500,math.cos(math.radians(observation[1])),math.sin(math.radians(observation[1])), math.radians(observation[3])]
        done = False
        steps = 0
        score = 0
        while not done:
            i_time = time.time()
            steps += 1
            if load_checkpoint:
                action = agent.choose_action(observation,deterministic=True)
            else:
                action = agent.choose_action(observation)

            observation_, reward, done, info = env.step(action)
            if (steps == 400 and not load_checkpoint):
                done = True
            observation_ = [observation_[0]/5000 , observation_[2]/500,math.cos(math.radians(observation_[1])),math.sin(math.radians(observation_[1])), math.radians(observation_[3])]
            score += reward
            agent.remember(observation, action, reward, observation_, done)
            env.render()
            observation = observation_
            
        
        score_history.append(score)
        avg_score = np.mean(score_history[-50:])
        env.center()
        if (not load_checkpoint):
            for j in range(steps):
                agent.learn()
        
        
        
        if avg_score > best_score:
            best_score = avg_score
            if not load_checkpoint:
                agent.save_models()
        print('episode ', i, 'score %.1f' % score, 'avg_score %.1f' % avg_score, 'best_score %.1f' % best_score)

    if not load_checkpoint:
        x = [i+1 for i in range(len(score_history))]
        plot_learning_curve(x, score_history, figure_file)