function [robot,homeConfig] = createRigidBodyTree
% Creates a robotics.RigidBodyTree object to be used by the 
% forward and inverse kinematics blocks.

% Copyright 2018 The MathWorks, Inc.
   robot = importrobot('Assem199.urdf');
    
    % Add gravity
   gravityVec = [0 0 -9.80665];
   robot.Gravity = gravityVec;
        
    % Add another massless coordinate frame for the end effector
   eeOffset = 0.3;
   eeBody = robotics.RigidBody('gripper_center');
   %eeBody.Mass = 0;
   %eeBody.Inertia = [0 0 0 0 0 0];
   setFixedTransform(eeBody.Joint,trvec2tform([0 eeOffset 0]));
   addBody(robot,eeBody,'gripper');

   % Return its home configuration
   homeConfig = robot.homeConfiguration;

end

