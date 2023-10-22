function [kittyMesh_h] = PlotCat
kittyCenterpoint = [-3 1 0];
[faces,vertex,data] = plyread('Cat.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
kittyVertexCount = 450;
kittyMesh_h = trisurf(faces,vertex(:,1)+kittyCenterpoint(1,1),vertex(:,2)+kittyCenterpoint(1,2), vertex(:,3)+kittyCenterpoint(1,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
end