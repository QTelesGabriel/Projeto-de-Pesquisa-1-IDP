import cv2
import sys

# Variável global para capturar o clique do mouse
ponto_clicado = None

def mouse_callback(event, x, y, flags, param):
    global ponto_clicado
    if event == cv2.EVENT_LBUTTONDOWN:
        ponto_clicado = (x, y)

def main():
    if len(sys.argv) < 2:
        print("\nComo usar: python3 calibrar_offset.py <caminho_da_foto>")
        sys.exit(1)
        
    caminho_imagem = sys.argv[1]
    img = cv2.imread(caminho_imagem)
    if img is None:
        print(f"Erro: Não foi possível abrir a imagem {caminho_imagem}")
        sys.exit(1)
        
    clone = img.copy()
    
    print("\n" + "="*50)
    print(" 1) DESENHE A BOUNDING BOX")
    print("="*50)
    print("Arraste o mouse para desenhar um quadrado ao redor de TODA a armadilha.")
    print("Pressione ENTER ou ESPAÇO para confirmar a caixa.")
    print("Pressione 'c' para cancelar a caixa e desenhar de novo.")
    
    bbox = cv2.selectROI("Calibrador de Offset", img, fromCenter=False, showCrosshair=True)
    
    if bbox[2] == 0 or bbox[3] == 0:
        print("\nVocê não desenhou a Bounding Box direito. Fechando o script.")
        sys.exit(1)
        
    x, y, w, h = bbox
    cx = x + w / 2.0
    cy = y + h / 2.0
    
    cv2.rectangle(clone, (int(x), int(y)), (int(x+w), int(y+h)), (255, 255, 0), 2)
    cv2.circle(clone, (int(cx), int(cy)), 5, (255, 255, 0), -1)
    
    cv2.imshow("Calibrador de Offset", clone)
    cv2.setMouseCallback("Calibrador de Offset", mouse_callback)
    
    print("\n" + "="*50)
    print(" 2) CLIQUE NO PONTO DE ALINHAMENTO")
    print("="*50)
    print("Clique na imagem no ponto exato onde o drone deveria centralizar (o gancho).")
    print("Pressione 'q' ou ESC após clicar para finalizar e ver os resultados.")
    
    global ponto_clicado
    while True:
        img_temp = clone.copy()
        if ponto_clicado is not None:
            px, py = ponto_clicado
            # Desenha o alvo clicado e uma linha do centro até ele
            cv2.circle(img_temp, (px, py), 8, (0, 0, 255), -1)
            cv2.line(img_temp, (int(cx), int(cy)), (px, py), (0, 255, 0), 2)
            
        cv2.imshow("Calibrador de Offset", img_temp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27: # ESC ou Q
            break
            
    if ponto_clicado is not None:
        px, py = ponto_clicado
        
        # Matemática da Proporção
        # X: (Posição Clicada - Centro) / Largura
        # Y: (Centro - Posição Clicada) / Altura  <-- Invertido porque no Pixel Y cresce para baixo
        offset_pct_x = (px - cx) / float(w)
        offset_pct_y = (cy - py) / float(h)
        
        print("\n" + "="*50)
        print(" RESULTADO DA CALIBRAÇÃO DE OFFSET")
        print("="*50)
        print(f"Largura da BB (w) : {w} px")
        print(f"Altura da BB  (h) : {h} px")
        print(f"Centro da BB (cx) : {cx:.1f} px")
        print(f"Centro da BB (cy) : {cy:.1f} px")
        print(f"Ponto de Alvo (px): {px} px")
        print(f"Ponto de Alvo (py): {py} px")
        print("-" * 50)
        print(" COPIE E COLE ESSES VALORES NO rastreador_fino.py:")
        print(f" self.offset_pct_x = {offset_pct_x:.4f}")
        print(f" self.offset_pct_y = {offset_pct_y:.4f}")
        print("="*50 + "\n")
    else:
        print("\nNenhum ponto foi clicado. Cancelando script.")
        
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
