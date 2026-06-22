import pandas as pd
import matplotlib.pyplot as plt
import os


HSV_FILE="../results/metrics/hsv.csv"
YOLO_FILE="../results/metrics/yolo.csv"

OUT="../results/analysis"

os.makedirs(
    OUT,
    exist_ok=True
)


def load(path,name):

    df=pd.read_csv(path)

    df["algorithm"]=name

    df["detected"]=(
        df["detected"]
        .astype(str)
        .str.lower()
        .map({
            "true":1,
            "false":0
        })
    )

    # tempo → latência
    df["latency_ms"]=df["time_ms"]

    # fps estimado
    df["fps"]=1000/df["latency_ms"]

    return df


hsv=load(
    HSV_FILE,
    "HSV"
)

yolo=load(
    YOLO_FILE,
    "YOLOv8n"
)

data=pd.concat(
    [
        hsv,
        yolo
    ]
)


#==================
# TABELA RESUMO
#==================

summary=[]

for algo in data.algorithm.unique():

    df=data[
        data.algorithm==algo
    ]

    total=len(df)

    detect=df.detected.sum()

    summary.append({

        "algorithm":
            algo,

        "images":
            total,

        "detection_rate":

            (
                detect
                /
                total
            )*100,

        "false_negative":

            (
                (
                    total-detect
                )
                /
                total
            )*100,

        "avg_latency_ms":

            df[
                "latency_ms"
            ].mean(),

        "avg_fps":

            df[
                "fps"
            ].mean(),

        "avg_confidence":

            df[
                "confidence"
            ].mean()
    })


summary=pd.DataFrame(
    summary
)

summary.to_csv(
    f"{OUT}/summary.csv",
    index=False
)

print("\nResumo:\n")
print(summary)


#==================
# FUNÇÃO DE GRÁFICO
#==================

def save_bar(
    values,
    ylabel,
    filename
):

    plt.figure(
        figsize=(6,5)
    )

    plt.bar(
        summary.algorithm,
        values
    )

    plt.ylabel(
        ylabel
    )

    plt.tight_layout()

    plt.savefig(
        f"{OUT}/{filename}"
    )

    plt.close()


save_bar(
    summary.detection_rate,
    "Taxa de Detecção (%)",
    "deteccao.png"
)

save_bar(
    summary.avg_latency_ms,
    "Latência Média (ms)",
    "latencia.png"
)

save_bar(
    summary.avg_fps,
    "FPS Médio",
    "fps.png"
)

save_bar(
    summary.avg_confidence,
    "Confiança Média",
    "confianca.png"
)


#==================
# CENÁRIOS
#==================

scenario=(

data

.groupby(
[
"scenario",
"algorithm"
]

)

.detected

.mean()

*100

)

scenario=scenario.unstack()

scenario.plot.bar(
    figsize=(8,5)
)

plt.ylabel(
    "Taxa de Detecção (%)"
)

plt.tight_layout()

plt.savefig(
    f"{OUT}/cenarios.png"
)

plt.close()


#==================
# TEXTO FINAL
#==================

best_det=summary.loc[
    summary
    .detection_rate
    .idxmax()
]

best_fps=summary.loc[
    summary
    .avg_fps
    .idxmax()
]

with open(
    f"{OUT}/conclusao.txt",
    "w"
) as f:

    f.write(
f"""
Maior taxa de detecção:
{best_det.algorithm}

Taxa:
{best_det.detection_rate:.2f}%

Melhor desempenho:
{best_fps.algorithm}

FPS:
{best_fps.avg_fps:.2f}

"""
)

print(
"\nConcluído."
)

print(
f"\nArquivos em {OUT}"
)