# syntax=rodrigojohann/cubagem-v1
FROM rodrigojohann/cubagem-v1:4.0
COPY src/cubagem-gui /home/cubagem-gui/src/cubagem-gui
WORKDIR /home/cubagem-gui/src
CMD ./cubagem-gui
