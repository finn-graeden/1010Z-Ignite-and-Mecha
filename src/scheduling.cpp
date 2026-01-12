#include <map>


struct Task {
    virtual void run() = 0;
    virtual ~Task() {}
};
std::map<Task*, int> tasks;
std::map<Task*, int> currentTasks;


void scheduleTask(Task* task, int perIteration){
    tasks[task] = perIteration;
    currentTasks[task] = perIteration;
}

void execute(){
    while (true){
        for (auto& pair : currentTasks){
            pair.second--;

            if (pair.second <= 0) {
                pair.first->run();
                pair.second = tasks[pair.first];
            }
        }
    }
}