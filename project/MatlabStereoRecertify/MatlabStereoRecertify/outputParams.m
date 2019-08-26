function [] = outputParams( stereoParams, imageWidth, imageHeight, xmlFileName )
%输出Matlab立体标定后的参数
    opencv_storage = com.mathworks.xml.XMLUtils.createDocument('opencv_storage');
    docRootNode = opencv_storage.getDocumentElement;

        imageSize = opencv_storage.createElement('imageSize'); %图像大小
        imageSize.appendChild(opencv_storage.createTextNode(sprintf('%i %i',imageWidth, imageHeight)));
        docRootNode.appendChild(imageSize);

        cameraMatrixL = opencv_storage.createElement('cameraMatrixL'); %左摄像头参数
        cameraMatrixL.setAttribute('type_id','opencv-matrix');%设置属性
        docRootNode.appendChild(cameraMatrixL);

            rows = opencv_storage.createElement('rows');
            rows.appendChild(opencv_storage.createTextNode(sprintf('3')));
            cameraMatrixL.appendChild(rows);

            cols = opencv_storage.createElement('cols');
            cols.appendChild(opencv_storage.createTextNode(sprintf('3')));
            cameraMatrixL.appendChild(cols);

            dt = opencv_storage.createElement('dt');
            dt.appendChild(opencv_storage.createTextNode(sprintf('d')));
            cameraMatrixL.appendChild(dt);

            data = opencv_storage.createElement('data');
            data.appendChild(opencv_storage.createTextNode(sprintf('%f %f %f %f %f %f %f %f %f',stereoParams.CameraParameters1.IntrinsicMatrix(1:9))));
            cameraMatrixL.appendChild(data);


        cameraDistcoeffL = opencv_storage.createElement('cameraDistcoeffL'); %左摄像头畸变参数
        cameraDistcoeffL.setAttribute('type_id','opencv-matrix');%设置属性
        docRootNode.appendChild(cameraDistcoeffL);

            rows = opencv_storage.createElement('rows');
            rows.appendChild(opencv_storage.createTextNode(sprintf('5')));
            cameraDistcoeffL.appendChild(rows);

            cols = opencv_storage.createElement('cols');
            cols.appendChild(opencv_storage.createTextNode(sprintf('1')));
            cameraDistcoeffL.appendChild(cols);

            dt = opencv_storage.createElement('dt');
            dt.appendChild(opencv_storage.createTextNode(sprintf('d')));
            cameraDistcoeffL.appendChild(dt);

            data = opencv_storage.createElement('data');
            data.appendChild(opencv_storage.createTextNode(sprintf('%f %f %f %f %f',stereoParams.CameraParameters1.RadialDistortion(1:2),stereoParams.CameraParameters1.TangentialDistortion(1:2),stereoParams.CameraParameters1.RadialDistortion(3))));
            cameraDistcoeffL.appendChild(data);

        cameraMatrixR = opencv_storage.createElement('cameraMatrixR'); %右摄像头参数
        cameraMatrixR.setAttribute('type_id','opencv-matrix');%设置属性
        docRootNode.appendChild(cameraMatrixR);

            rows = opencv_storage.createElement('rows');
            rows.appendChild(opencv_storage.createTextNode(sprintf('3')));
            cameraMatrixR.appendChild(rows);

            cols = opencv_storage.createElement('cols');
            cols.appendChild(opencv_storage.createTextNode(sprintf('3')));
            cameraMatrixR.appendChild(cols);

            dt = opencv_storage.createElement('dt');
            dt.appendChild(opencv_storage.createTextNode(sprintf('d')));
            cameraMatrixR.appendChild(dt);

            data = opencv_storage.createElement('data');
            data.appendChild(opencv_storage.createTextNode(sprintf('%f %f %f %f %f %f %f %f %f',stereoParams.CameraParameters2.IntrinsicMatrix(1:9))));
            cameraMatrixR.appendChild(data);


        cameraDistcoeffR = opencv_storage.createElement('cameraDistcoeffR'); %右摄像头畸变参数
        cameraDistcoeffR.setAttribute('type_id','opencv-matrix');%设置属性
        docRootNode.appendChild(cameraDistcoeffR);

            rows = opencv_storage.createElement('rows');
            rows.appendChild(opencv_storage.createTextNode(sprintf('5')));
            cameraDistcoeffR.appendChild(rows);

            cols = opencv_storage.createElement('cols');
            cols.appendChild(opencv_storage.createTextNode(sprintf('1')));
            cameraDistcoeffR.appendChild(cols);

            dt = opencv_storage.createElement('dt');
            dt.appendChild(opencv_storage.createTextNode(sprintf('d')));
            cameraDistcoeffR.appendChild(dt);

            data = opencv_storage.createElement('data');
            data.appendChild(opencv_storage.createTextNode(sprintf('%f %f %f %f %f',stereoParams.CameraParameters2.RadialDistortion(1:2),stereoParams.CameraParameters2.TangentialDistortion(1:2),stereoParams.CameraParameters2.RadialDistortion(3))));
            cameraDistcoeffR.appendChild(data);


        R = opencv_storage.createElement('R'); %旋转矩阵
        R.setAttribute('type_id','opencv-matrix');%设置属性
        docRootNode.appendChild(R);

            rows = opencv_storage.createElement('rows');
            rows.appendChild(opencv_storage.createTextNode(sprintf('3')));
            R.appendChild(rows);

            cols = opencv_storage.createElement('cols');
            cols.appendChild(opencv_storage.createTextNode(sprintf('3')));
            R.appendChild(cols);

            dt = opencv_storage.createElement('dt');
            dt.appendChild(opencv_storage.createTextNode(sprintf('d')));
            R.appendChild(dt);

            data = opencv_storage.createElement('data'); 
            data.appendChild(opencv_storage.createTextNode(sprintf('%f %f %f %f %f %f %f %f %f',stereoParams.RotationOfCamera2(1:9))));
            R.appendChild(data);      

        T = opencv_storage.createElement('T'); %平移矩阵
        T.setAttribute('type_id','opencv-matrix');%设置属性
        docRootNode.appendChild(T);

            rows = opencv_storage.createElement('rows');
            rows.appendChild(opencv_storage.createTextNode(sprintf('3')));
            T.appendChild(rows);

            cols = opencv_storage.createElement('cols');
            cols.appendChild(opencv_storage.createTextNode(sprintf('1')));
            T.appendChild(cols);

            dt = opencv_storage.createElement('dt');
            dt.appendChild(opencv_storage.createTextNode(sprintf('d')));
            T.appendChild(dt);

            data = opencv_storage.createElement('data');
            data.appendChild(opencv_storage.createTextNode(sprintf('%f %f %f',stereoParams.TranslationOfCamera2(1:3))));
            T.appendChild(data);      

    opencv_storage.appendChild(opencv_storage.createComment('2016.10.1'));

    xmlwrite(xmlFileName,opencv_storage);
    type(xmlFileName);

end

