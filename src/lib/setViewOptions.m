function setViewOptions(limits)
    
    title({ ["TP1 - Industrial Robotics"],["Bruno Silva 98374"]})
    subtitle("Press a key to start")
    fontname("Fira Code SemiBold")

    view(135, 25)
    grid on
    axis equal
    xlabel('x'); ylabel('y'); zlabel('z')

    ax = gca;

    ax.Clipping = "off";
    
    axis(limits)

end
