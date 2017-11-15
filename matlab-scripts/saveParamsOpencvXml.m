function [  ] = saveParamsOpencvXml( param, file_name )

    % Create a sample XML document.
    docNode = com.mathworks.xml.XMLUtils.createDocument('opencv_storage');
    docRootNode = docNode.getDocumentElement;
    
    NUM_PROJ = (length(param) - 13) / 10; % N*10(projs) + 4(sphere) + 9(cam)
    
    for i = 1:NUM_PROJ
       elemName = ['proj' num2str(i - 1) '_param']; 
       elemNode = docNode.createElement(elemName);
       elemNode.setAttribute('type_id','opencv-matrix');
       docRootNode.appendChild(elemNode);
       
       subElemNode = docNode.createElement('rows');
       subElemNode.appendChild(docNode.createTextNode(num2str(10)));
       elemNode.appendChild(subElemNode);       
       subElemNode = docNode.createElement('cols');
       subElemNode.appendChild(docNode.createTextNode(num2str(1)));
       elemNode.appendChild(subElemNode);        
       subElemNode = docNode.createElement('dt');
       subElemNode.appendChild(docNode.createTextNode('f'));
       elemNode.appendChild(subElemNode); 
       
       subElemNode = docNode.createElement('data');
       paramRange = [1:10] + (i-1)*10;
       subElemNode.appendChild(docNode.createTextNode(num2str(param(paramRange)', '%10.8e ')));
       elemNode.appendChild(subElemNode);              
    end
    
       elemName = ['sphere_pose']; 
       elemNode = docNode.createElement(elemName);
       elemNode.setAttribute('type_id','opencv-matrix');
       docRootNode.appendChild(elemNode);
       
       subElemNode = docNode.createElement('rows');
       subElemNode.appendChild(docNode.createTextNode(num2str(4)));
       elemNode.appendChild(subElemNode);       
       subElemNode = docNode.createElement('cols');
       subElemNode.appendChild(docNode.createTextNode(num2str(1)));
       elemNode.appendChild(subElemNode);        
       subElemNode = docNode.createElement('dt');
       subElemNode.appendChild(docNode.createTextNode('f'));
       elemNode.appendChild(subElemNode); 
       
       subElemNode = docNode.createElement('data');
       paramRange = [1:4] + NUM_PROJ*10;
       subElemNode.appendChild(docNode.createTextNode(num2str(param(paramRange)', '%10.8e ')));
       elemNode.appendChild(subElemNode);    

       elemName = ['cam_param']; 
       elemNode = docNode.createElement(elemName);
       elemNode.setAttribute('type_id','opencv-matrix');
       docRootNode.appendChild(elemNode);
       
       subElemNode = docNode.createElement('rows');
       subElemNode.appendChild(docNode.createTextNode(num2str(9)));
       elemNode.appendChild(subElemNode);       
       subElemNode = docNode.createElement('cols');
       subElemNode.appendChild(docNode.createTextNode(num2str(1)));
       elemNode.appendChild(subElemNode);        
       subElemNode = docNode.createElement('dt');
       subElemNode.appendChild(docNode.createTextNode('f'));
       elemNode.appendChild(subElemNode); 
       
       subElemNode = docNode.createElement('data');
       paramRange = [5:13] + NUM_PROJ*10;
       subElemNode.appendChild(docNode.createTextNode(num2str(param(paramRange)', '%10.8e ')));
       elemNode.appendChild(subElemNode);       
       
    xmlFileName = [file_name,'.xml'];
    xmlwrite(xmlFileName,docNode);
end

