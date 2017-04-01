

addpath('function2');
addpath('E:\Code\coding\MyCode\data\depth');

img = depthRead('1.png');

figure, imshow(img,[])
title('Ô­Ê¼Í¼Ïñ')

resultI = BilateralFiltGray(double(img), d, sigma);

figure, imshow(resultI,[])
title('Ë«±ßÂË²¨ºóµÄÍ¼Ïñ')

