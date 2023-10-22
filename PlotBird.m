function [BrickMesh_h] = PlotBird(xPos)
bricknum = [xPos,-1.5,0.1];
[f,v,data] = plyread('shuttleCock.PLY','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
BrickVertexCount = size(v,1);
BrickMesh_h = trisurf(f,v(:,1)+bricknum(1,1),v(:,2)+bricknum(1,2), v(:,3)+bricknum(1,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
end