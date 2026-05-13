import cv2
import numpy as np

def main():
    cap = cv2.VideoCapture(0)

    print("Detectando cor azul. Pressione 'q' para sair.")

    while True:
        success, frame = cap.read()
        if not success:
            break

        # 1. Converte o frame de BGR (padrão do OpenCV) para HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Define o intervalo da cor azul no espaço HSV
        # Nota: O matiz (Hue) para azul geralmente fica em torno de 110-130
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        # 3. Cria uma máscara: pixels dentro do intervalo ficam brancos (255), fora ficam pretos (0)
        mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

        # 4. Encontra os contornos da área branca na máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            # Filtra por área para ignorar pequenos ruídos
            if cv2.contourArea(cnt) > 500:
                # 5. Calcula a Bounding Box (retângulo envolvente)
                x, y, w, h = cv2.boundingRect(cnt)
                
                # Desenha o retângulo no frame original
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, "Azul detectado", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Exibe os resultados
        cv2.imshow("Original com Bounding Box", frame)
        cv2.imshow("Mascara (Preto e Branco)", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()