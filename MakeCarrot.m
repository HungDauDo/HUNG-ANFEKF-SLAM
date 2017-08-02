function MakeCarrot( real_position,ScaleFactor )
%Make a carrot ^ indicating the orientation of the robot
    %rot_matrix = [cos(avg_robot_position(3)+pi/2)) -sin(avg_robot_position(3)+pi/2);sin(avg_robot_position(3)+pi/2) cos(avg_robot_position(3)+pi/2)];
    real_position(3)= -real_position(3)*5;
    pt1=[real_position(1)-ScaleFactor;real_position(2)];
    pt2=[real_position(1);real_position(2)+ScaleFactor];
    pt3=[real_position(1)+ScaleFactor;real_position(2)];
    
    pt1=[real_position(1)+(pt1(1)-real_position(1))*cos(real_position (3))+(pt1(2)-real_position(2))*sin(real_position (3));...
        real_position(2) - (pt1(1)-real_position(1))*sin(real_position (3)) + (pt1(2)-real_position(2))*cos(real_position (3))];
    pt2=[real_position(1)+(pt2(1)-real_position(1))*cos(real_position (3))+(pt2(2)-real_position(2))*sin(real_position (3));...
        real_position(2) - (pt2(1)-real_position(1))*sin(real_position (3)) + (pt2(2)-real_position(2))*cos(real_position (3))];
    pt3=[real_position(1)+(pt3(1)-real_position(1))*cos(real_position (3))+(pt3(2)-real_position(2))*sin(real_position (3));...
        real_position(2) - (pt3(1)-real_position(1))*sin(real_position (3)) + (pt3(2)-real_position(2))*cos(real_position (3))];
    
    line([pt1(1) pt2(1)],[pt1(2) pt2(2)],'Color','k');
    line([pt2(1) pt3(1)],[pt2(2) pt3(2)],'Color','k');
end