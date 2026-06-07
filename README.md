
# pegador_de_bandeiras

*Projeto da disciplina SSC0712 - Programação de Robôs Móveis*

Este projeto é um robô móvel autônomo capaz de se deslocar em uma arena com obstáculos até uma bandeira e depois alinhar a garra ao mastro da bandeira, tudo isso em simulação utilizando o Gazebo. O robô funciona a partir de uma máquina de estados, com os seguintes estados implementados até o momento:
- EXPLORANDO: o robô anda para frente. Se encontrar um obstáculo, alterna para o estado "DESVIANDO_DE_OBSTACULO", e se encontrar a bandeira alterna para o estado "NAVEGANDO_PARA_BANDEIRA"

- DESVIANDO_DE_OBSTACULO: o robô para e vira na direção oposta do obstáculo para desviar. Depois, ele anda um pouco para frente e muda para o modo "EXPLORANDO".

- NAVEGANDO_PARA_BANDEIRA: o robô vai para frente, porém constantemente alinhando sua trajetória na direção da bandeira. Se ele perder a bandeira de vista, retorna para o estado "EXPLORANDO", e se encontrar um obstáculo ele muda para o estado "DESVIANDO_DE_OBSTACULO". Caso contrário, se o robô fica suficientemente próximo da bandeira, alterna para o modo "POSICIONANDO_PARA_COLETA".

- POSICIONANDO_PARA_COLETA: o robô se alinha com maior precisão ao mastro da bandeira e depois se aproxima o máximo possível. Ao ficar o mais alinhado e próximo da bandeira o possível, muda para o modo "CAPTURANDO_BANDEIRA".

- CAPTURANDO_BANDEIRA: ainda não implementado.

Diagrama de Estados:
![Diagrama de Estados](https://github.com/jp-lopes/pegador_de_bandeiras/blob/pegador_de_bandeiras_mk1/Diagrama_de_estados.png)



## Autores
- Andre Luiz de Souza Murakami - nUSP 5631500 - [@Andre-Murakami](https://github.com/Andre-Murakami)
- Caio Cesar Trentin de Assis - nUSP 15674233 - [@CaioCesarTA](https://github.com/CaioCesarTA)
- João Pedro Lopes de Melo - nUSP 15588950 - [@jp-lopes](https://github.com/jp-lopes)


## Instruções para execução localmente

1. Acessar a pasta `src` do seu workspace ROS2 Humble e clonar o repositório:
    ```bash 
    cd ~/ros2_ws/src
    git clone -b pegador_de_bandeiras_mk1 https://github.com/jp-lopes/pegador_de_bandeiras.git pegador_de_bandeiras_mk1
    ```
2. Instalar dependências com `rosdep`:
    ```bash 
    cd ~/ros2_ws
    sudo apt update
    sudo rosdep init        
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Compilar o pacote:
    ```bash 
    cd ~/ros2_ws
    colcon build
    ```
4. Iniciar a simulação do Gazebo:
    ```bash 
    source install/setup.bash
    ros2 launch pegador_de_bandeiras_mk1 inicia_simulacao.launch.py
    ```
5. Abrir mais dois terminais:
- No primeiro, carregar o robô na simulação:
    ```bash 
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch pegador_de_bandeiras_mk1 carrega_robo.launch.py
    ```
- No segundo, iniciar o controle autonômo do robô:
    ```bash 
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run pegador_de_bandeiras_mk1 controle_robo
    ```
## Instruções para execução com Docker
1. Acessar a pasta `src` do seu workspace ROS2 Humble e clonar o repositório:
    ```bash 
    cd ~/ros2_ws/src
    git clone -b pegador_de_bandeiras_mk1 https://github.com/jp-lopes/pegador_de_bandeiras.git pegador_de_bandeiras_mk1
    ```
2. Garantir permissões gráficas e iniciar container:
    ```bash 
    cd pegador_de_bandeiras_mk1/docker
    xhost +local:root
    docker compose up -d
    ```
3. Entrar no container, compilar projeto e carregar variáveis:
    ```bash 
    docker exec -it ros2_humble_env bash
    colcon build
    source ~/.bashrc
    ```
4. Iniciar simulação:
    ```bash 
    ros2 launch pegador_de_bandeiras_mk1 inicia_simulacao.launch.py
    ```
5. Abrir mais dois terminais:
- No primeiro, carregar robô na simulação:
    ```bash 
    docker exec -it ros2_humble_env bash
    ros2 launch pegador_de_bandeiras_mk1 carrega_robo.launch.py
    ```
- No segundo, iniciar o controle autonômo do robô:
    ```bash 
    docker exec -it ros2_humble_env bash
    ros2 run pegador_de_bandeiras_mk1 controle_robo
    ```
