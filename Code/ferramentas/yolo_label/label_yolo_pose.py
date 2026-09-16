import cv2
import os
import glob
import sys

# ================================
# CONFIGURAÇÕES
# ================================
IMAGES_DIR = "dataset/imagens"
LABELS_DIR = "dataset/labels"
CLASS_ID = 0

# QUANTOS PONTOS A SUA ARMADILHA VAI TER? (Ex: 4 cantos superiores)
NUM_KEYPOINTS = 4 

os.makedirs(LABELS_DIR, exist_ok=True)

keypoints_clicked = []

def mouse_callback(event, x, y, flags, param):
    global keypoints_clicked
    
    if len(keypoints_clicked) >= NUM_KEYPOINTS:
        return
        
    # BOTÃO ESQUERDO: Ponto Visível
    if event == cv2.EVENT_LBUTTONDOWN:
        keypoints_clicked.append((x, y, 2.0)) # 2.0 = Visível
        
    # BOTÃO DIREITO: Ponto Ausente/Invisível (Fora da tela)
    elif event == cv2.EVENT_RBUTTONDOWN:
        keypoints_clicked.append((0, 0, 0.0)) # 0.0 = Ausente
        
    if event in [cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN]:
        img_temp = param.copy()
        
        # Redesenha todos os pontos
        for px, py, v in keypoints_clicked:
            if v == 2.0:
                cv2.circle(img_temp, (px, py), 5, (0, 0, 255), -1)
            # Se for ausente (v=0), não desenha bolinha
        
        faltam = NUM_KEYPOINTS - len(keypoints_clicked)
        if faltam > 0:
            cv2.putText(img_temp, f"Faltam {faltam}. Esq=Visivel | Dir=Invisivel/Ausente", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            cv2.putText(img_temp, "Pontos registrados! Aperte ESPACO para continuar.", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        cv2.imshow("Rotulagem Pose", img_temp)

# ================================
# Coletar e ordenar imagens
# ================================
image_paths = []
for ext in ["*.jpg", "*.png", "*.jpeg"]:
    image_paths.extend(glob.glob(os.path.join(IMAGES_DIR, ext)))

image_paths.sort()

if not image_paths:
    print(f"Nenhuma imagem encontrada em '{IMAGES_DIR}'.")
    sys.exit()

print(f"Total de imagens encontradas: {len(image_paths)}")
print("-" * 50)
print("INSTRUÇÕES DE USO (YOLO-POSE):")
print("1. Desenhe a caixa inteira da armadilha e aperte ENTER.")
print(f"2. A imagem vai congelar. Registre as {NUM_KEYPOINTS - 1} alças:")
print("   - BOTÃO ESQUERDO: Clica na alça (Se ela estiver na tela).")
print("   - BOTÃO DIREITO: Marca a alça como INVISÍVEL (Se o drone desceu muito e ela sumiu).")
print("3. Quando acabar os cliques, aperte ENTER para ir para a próxima foto.")
print("- O programa pulará automaticamente as fotos já rotuladas.")
print("-" * 50)

# ================================
# Loop de Rotulagem
# ================================
for i, image_path in enumerate(image_paths):
    image_name = os.path.splitext(os.path.basename(image_path))[0]
    label_path = os.path.join(LABELS_DIR, image_name + ".txt")

    # Pula a imagem se o label já existir
    if os.path.exists(label_path):
        print(f"[{i+1}/{len(image_paths)}] Pulando: {os.path.basename(image_path)} (Já rotulado)")
        continue

    image = cv2.imread(image_path)
    if image is None:
        continue

    img_h, img_w = image.shape[:2]
    boxes = []

    print(f"\n[{i+1}/{len(image_paths)}] Rotulando: {os.path.basename(image_path)}")

    while True:
        # Passo 1: O usuário seleciona a Bounding Box (Caixa)
        bbox = cv2.selectROI("Rotulagem Pose", image, fromCenter=False)
        x, y, w, h = bbox

        # Se o usuário apertar ENTER sem desenhar nada (w=0, h=0), significa que acabou a foto.
        if w == 0 or h == 0:
            break

        # Passo 2: O usuário clica os Keypoints
        keypoints_clicked = []
        
        # --- PONTO CENTRAL AUTOMÁTICO (DA CÂMERA) ---
        centro_camera_x = int(img_w / 2.0)
        centro_camera_y = int(img_h / 2.0)
        # O centro sempre vai estar visível (v=2.0)
        keypoints_clicked.append((centro_camera_x, centro_camera_y, 2.0))
        
        img_copy = image.copy()
        cv2.rectangle(img_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Desenha o 1º ponto em AZUL
        cv2.circle(img_copy, (centro_camera_x, centro_camera_y), 6, (255, 0, 0), -1)
        
        cv2.putText(img_copy, f"Centro salvo! Registre as {NUM_KEYPOINTS - 1} alcas (Esq=Visivel / Dir=Invisivel)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        cv2.imshow("Rotulagem Pose", img_copy)
        cv2.setMouseCallback("Rotulagem Pose", mouse_callback, img_copy)
        
        print(f"   -> Centro Registrado. Aguardando você registrar as {NUM_KEYPOINTS - 1} alcas...")
        
        # Fica travado num laço até o usuário clicar a quantidade certa de vezes E apertar ENTER/ESPACO
        while True:
            k = cv2.waitKey(10)
            if len(keypoints_clicked) == NUM_KEYPOINTS and k in [13, 32]:
                break

        # Tira o evento de clique do mouse
        cv2.setMouseCallback("Rotulagem Pose", lambda *args: None) 
        
        boxes.append((x, y, w, h, keypoints_clicked))
        
        # Desenha na imagem de fundo para feedback visual
        for px, py, v in keypoints_clicked:
            if v == 2.0:
                cv2.circle(image, (px, py), 5, (0, 0, 255), -1)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # ==========================
    # Salvar YOLO Pose (Formato: ID x_box y_box w_box h_box px1 py1 v1 px2 py2 v2...)
    # Tudo normalizado entre 0 e 1, 'v' é a visibilidade (0 = Ausente, 2 = Visível)
    # ==========================
    with open(label_path, "w") as f:
        for x, y, w, h, kpts in boxes:
            x_center = (x + w/2) / img_w
            y_center = (y + h/2) / img_h
            width = w / img_w
            height = h / img_h

            line = f"{CLASS_ID} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}"
            
            for px, py, v in kpts:
                if v == 0.0:
                    # Ponto invisível (Drone muito perto e alça sumiu da tela)
                    line += " 0.000000 0.000000 0.000000"
                else:
                    # Ponto visível
                    norm_x = px / img_w
                    norm_y = py / img_h
                    line += f" {norm_x:.6f} {norm_y:.6f} 2.000000"
                
            f.write(line + "\n")

    if not boxes:
        print(f"[SALVO] {label_path} (Background / Vazio)")
    else:
        print(f"[SALVO] {label_path}")

cv2.destroyAllWindows()
print("\nProcesso de rotulagem de POSE concluído com sucesso!")
