# barrett_trajectory_action_server

This controller satisfies the ros control interface to followJointTrajectorys so that this package can work with MoveIt!.  The controller will follow a given trajectory via a guarded move. Each finger will stop independently if the finger's tactile sensors record contact above a threshold.  This node also exposes an interface in which clients can query the hand to learn where contact occurred after execution. 
