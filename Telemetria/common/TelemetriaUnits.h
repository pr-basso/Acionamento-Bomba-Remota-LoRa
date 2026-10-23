/**
 * ============================================================
 *  REDE DE TELEMETRIA — UNIDADES DAS CHAVES DO PAYLOAD
 *
 *  Só para exibição (o MQTT leva o número puro). Compartilhado
 *  por link simbólico entre o Gateway (OLED) e o Display (LCD),
 *  que roda em outra placa — por isso fica fora do
 *  TelemetriaConfig.h, que exige a Heltec V4.
 *  Chave fora da tabela aparece sem unidade.
 * ============================================================
 */

#ifndef TELEMETRIA_UNITS_H
#define TELEMETRIA_UNITS_H

#include <string.h>

struct Unit { const char *key; const char *unit; };
static const Unit UNITS[] = {
  { "pct",   "%"  }, { "dist", "cm" }, { "nivel", "cm" },
  { "temp",  "°C" }, { "umid", "%"  },
  { "va",    "V"  }, { "vb",   "V"  }, { "vc",    "V"  },
  { "vbat",  "V"  },
};

static const char *unitFor(const char *key) {
  for (const Unit &u : UNITS) if (strcmp(u.key, key) == 0) return u.unit;
  return "";
}

#endif
