"""CSS and minimal JS for observation HTML export reports."""

REPORT_CSS = """
:root{
  --bg:#e8edf4;--bg2:#f8fafc;--card:#fff;--border:#d1dae6;--border-light:#e8edf3;
  --text:#0f172a;--muted:#64748b;--header:#0c1222;--accent:#2563eb;
  --good:#15803d;--warn:#b45309;--bad:#b91c1c;--target:#1d4ed8;--impact:#15803d;
  --corr:#c2410c;--dem:#0369a1;--teal:#0d9488;--radius:14px;--radius-sm:10px;
  --shadow:0 1px 2px rgba(15,23,42,.05),0 8px 24px rgba(15,23,42,.07);
  --shadow-sm:0 1px 3px rgba(15,23,42,.08);
  --font:Segoe UI,system-ui,-apple-system,BlinkMacSystemFont,Arial,sans-serif;
  --mono:Consolas,Monaco,ui-monospace,monospace;
}
*,*::before,*::after{box-sizing:border-box;}
html{scroll-behavior:smooth;}
body{
  margin:0;background:linear-gradient(165deg,var(--bg2) 0%,var(--bg) 100%);
  color:var(--text);font-family:var(--font);line-height:1.5;
  -webkit-font-smoothing:antialiased;
}
.report-page{max-width:1180px;margin:0 auto;padding:20px 16px 48px;}
.report-header{
  background:linear-gradient(135deg,#0c1222 0%,#1e3a5f 55%,#0f4c5c 100%);
  color:#f8fafc;border-radius:var(--radius);padding:24px 28px;margin-bottom:0;
  box-shadow:var(--shadow);position:relative;overflow:hidden;
}
.report-header::after{
  content:'';position:absolute;top:-40%;right:-10%;width:45%;height:180%;
  background:radial-gradient(circle,rgba(45,212,191,.15) 0%,transparent 70%);
  pointer-events:none;
}
.report-header-inner{display:flex;flex-wrap:wrap;gap:20px;align-items:flex-start;
  justify-content:space-between;position:relative;z-index:1;}
.report-header-text{flex:1;min-width:200px;}
.report-brand{font-size:11px;font-weight:700;text-transform:uppercase;
  letter-spacing:.12em;color:#94a3b8;margin-bottom:6px;}
.report-header h1{margin:0 0 8px;font-size:1.5rem;font-weight:700;letter-spacing:-.02em;}
.report-meta{display:flex;flex-wrap:wrap;gap:8px 16px;font-size:13px;color:#cbd5e1;}
.report-meta-pill{
  display:inline-flex;align-items:center;gap:6px;padding:4px 10px;
  background:rgba(255,255,255,.08);border-radius:999px;border:1px solid rgba(255,255,255,.12);
}
.report-meta-pill strong{color:#f8fafc;font-weight:600;}
.hero-kpis{display:flex;flex-wrap:wrap;gap:10px;align-items:stretch;}
.hero-kpi{
  min-width:100px;padding:12px 16px;border-radius:var(--radius-sm);
  background:rgba(255,255,255,.1);border:1px solid rgba(255,255,255,.15);
  backdrop-filter:blur(8px);text-align:center;
}
.hero-kpi-val{display:block;font-size:1.35rem;font-weight:800;line-height:1.2;color:#fff;}
.hero-kpi-label{display:block;font-size:10px;text-transform:uppercase;
  letter-spacing:.06em;color:#94a3b8;margin-top:4px;font-weight:600;}
.hero-kpi-miss .hero-kpi-val{color:#fdba74;}
.hero-kpi-range .hero-kpi-val{color:#7dd3fc;}
.hero-kpi-defl .hero-kpi-val{color:#c4b5fd;}
.report-nav-wrap{
  position:sticky;top:0;z-index:200;margin:14px 0 20px;
  background:rgba(248,250,252,.92);backdrop-filter:blur(12px);
  border:1px solid var(--border);border-radius:var(--radius-sm);
  box-shadow:var(--shadow-sm);padding:6px 10px;
}
.report-nav{display:flex;flex-wrap:wrap;gap:4px;align-items:center;}
.report-nav a{
  display:inline-block;padding:8px 14px;font-size:12px;font-weight:600;
  color:var(--muted);text-decoration:none;border-radius:8px;transition:all .15s ease;
}
.report-nav a:hover{background:#e2e8f0;color:var(--text);}
.report-nav a.is-active{background:var(--accent);color:#fff;}
.section-card{
  background:var(--card);border:1px solid var(--border-light);border-radius:var(--radius);
  margin-bottom:18px;box-shadow:var(--shadow-sm);overflow:hidden;
}
.section-head{
  padding:16px 20px 12px;border-bottom:1px solid var(--border-light);
  background:linear-gradient(180deg,#fafbfc 0%,#fff 100%);
}
.section-body{padding:16px 20px 20px;}
.section-title{margin:0;font-size:1rem;font-weight:700;color:var(--text);}
.section-subtitle{margin:6px 0 0;font-size:13px;color:var(--muted);line-height:1.45;}
.dooaf-client-corr .section-head{background:linear-gradient(180deg,#fff7ed 0%,#fff 100%);}
.dooaf-client-corr{border-color:#fed7aa;}
.dooaf-fire-corr{border-color:#fed7aa;}
.dooaf-fire-corr .section-head{background:linear-gradient(180deg,#fff7ed 0%,#fff 100%);}
.section-technical .section-title::after{
  content:' — optional';font-weight:500;font-size:.72rem;color:var(--muted);margin-left:6px;
}
.data-table{width:100%;border-collapse:separate;border-spacing:0;font-size:13px;
  border:1px solid var(--border-light);border-radius:var(--radius-sm);overflow:hidden;}
.data-table th,.data-table td{border-bottom:1px solid var(--border-light);
  padding:10px 14px;vertical-align:top;text-align:left;}
.data-table thead th{
  background:#f1f5f9;color:#475569;font-weight:600;font-size:11px;
  text-transform:uppercase;letter-spacing:.05em;border-bottom:2px solid var(--border);
}
.data-table tbody tr:last-child td{border-bottom:none;}
.data-table tbody tr:hover td{background:#f8fafc;}
.data-table .label-col{font-weight:600;color:#334155;width:30%;}
.mono{font-family:var(--mono);font-size:12px;}
.muted{color:var(--muted);}
.table-scroll{margin-top:4px;border-radius:var(--radius-sm);overflow:auto;max-width:100%;}
.badge{display:inline-block;padding:3px 10px;border-radius:999px;font-size:11px;
  font-weight:600;line-height:1.5;white-space:nowrap;}
.badge-good{background:#dcfce7;color:var(--good);}
.badge-bad{background:#fee2e2;color:var(--bad);}
.badge-warn{background:#ffedd5;color:var(--warn);}
.badge-info{background:#e0f2fe;color:#0369a1;}
.badge-dem{background:#e0f2fe;color:var(--dem);border:1px solid #7dd3fc;}
.badge-muted{background:#f1f5f9;color:var(--muted);}
.dooaf-target-coords td{color:var(--target);font-weight:600;background:#eff6ff;}
.dooaf-impact-coords td{color:var(--impact);font-weight:600;background:#ecfdf5;}
.metrics-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;margin-bottom:14px;}
.metric-card{background:#fff;border:1px solid #fdba74;border-radius:var(--radius-sm);padding:14px 16px;}
.metric-label{font-size:10px;color:var(--muted);text-transform:uppercase;letter-spacing:.05em;margin-bottom:6px;}
.metric-value{font-size:1.5rem;font-weight:800;color:var(--corr);line-height:1.2;}
.metric-sub{font-size:11px;color:var(--muted);margin-top:4px;}
.camera-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px;}
.camera-stat{background:#f8fafc;border:1px solid var(--border-light);border-radius:var(--radius-sm);padding:14px 16px;}
.camera-stat .label{font-size:10px;color:var(--muted);text-transform:uppercase;letter-spacing:.05em;margin-bottom:8px;}
.camera-stat .value{font-size:15px;font-weight:600;}
.log-entries{display:flex;flex-direction:column;gap:16px;}
.log-entry{border:1px solid var(--border-light);border-radius:var(--radius);background:#fff;overflow:hidden;}
.log-entry-impact{border-color:#86efac;box-shadow:0 0 0 1px rgba(74,222,128,.2);}
.log-entry-head{display:flex;flex-wrap:wrap;align-items:center;gap:8px 12px;padding:14px 18px;
  background:#f8fafc;border-bottom:1px solid var(--border-light);}
.log-entry-impact .log-entry-head{background:linear-gradient(90deg,#ecfdf5 0%,#f8fafc 100%);}
.log-entry-index{font-weight:700;font-size:13px;}
.log-entry-time{font-size:12px;color:var(--muted);font-family:var(--mono);}
.log-entry-badges{display:flex;flex-wrap:wrap;gap:6px;margin-left:auto;}
.log-detail-table{width:100%;border-collapse:collapse;font-size:12px;}
.log-detail-table th{width:34%;padding:10px 18px;font-weight:600;color:#475569;
  background:#fafbfc;border-bottom:1px solid var(--border-light);vertical-align:top;}
.log-detail-table td{padding:10px 18px;border-bottom:1px solid var(--border-light);vertical-align:top;}
.log-detail-section td{background:#f1f5f9;color:#334155;font-size:10px;font-weight:700;
  text-transform:uppercase;letter-spacing:.06em;padding:8px 18px;}
.log-detail-table tbody tr:last-child th,.log-detail-table tbody tr:last-child td{border-bottom:none;}
.mgrs-badge{display:inline-block;font-family:var(--mono);font-size:12px;padding:5px 10px;
  background:#e2e8f0;border-radius:6px;white-space:nowrap;color:#334155;}
.elev-badge{display:inline-block;font-family:var(--mono);font-size:12px;padding:5px 10px;
  background:#fef3c7;border-radius:6px;color:#92400e;border:1px solid #fcd34d;}
.coord-pair{cursor:help;border-bottom:1px dotted #94a3b8;}
.kind-badge{background:#e0e7ff;color:#3730a3;}
.role-badge{background:#f1f5f9;color:#475569;}
.file-link{color:var(--accent);text-decoration:none;}
.file-link:hover{text-decoration:underline;}
.log-hint{font-size:13px;color:var(--muted);margin:0 0 14px;line-height:1.55;}
.fc-legend{display:flex;flex-wrap:wrap;gap:10px 16px;margin:0 0 14px;padding:10px 14px;
  background:#f8fafc;border-radius:var(--radius-sm);border:1px solid var(--border-light);font-size:12px;color:var(--muted);}
.fc-dot{display:inline-block;width:11px;height:11px;border-radius:50%;vertical-align:middle;
  margin-right:6px;border:2px solid #fff;box-shadow:0 0 0 1px rgba(0,0,0,.1);}
.fc-dot-gun{background:#2563eb;}.fc-dot-target{background:#16a34a;}
.fc-dot-impact{background:#dc2626;}.fc-dot-drone{background:#9333ea;}
.fc-diagram-grid,.fc-diagram-grid-4{display:grid;grid-template-columns:1fr 1fr;gap:16px;margin:14px 0;align-items:start;}
.fc-diagram-grid .viz-card,.fc-diagram-grid-4 .viz-card{align-self:start;}
.fc-diagram-wrap svg{display:block;vertical-align:top;}
.viz-card{background:#fff;border:1px solid var(--border-light);border-radius:var(--radius-sm);
  overflow:hidden;box-shadow:var(--shadow-sm);}
.viz-card-head{padding:10px 14px;background:#f8fafc;border-bottom:1px solid var(--border-light);
  font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.06em;color:var(--muted);}
.viz-card-body{padding:8px;}
.fc-diagram-wrap{background:#fafbfc;border:1px solid var(--border-light);border-radius:var(--radius-sm);padding:8px;}
.fc-diagram-wrap svg{display:block;vertical-align:top;min-height:320px;}
.fc-positions-svg,.fc-plan-svg,.fc-gunline-svg,.fc-compass-svg{min-height:360px;}
.lr-icon{display:inline-block;min-width:1.15em;font-weight:800;text-align:center;margin-right:4px;}
.lr-pos{color:#15803d;}
.lr-neg{color:#b91c1c;}
.fc-diagram-title{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.05em;
  color:var(--muted);text-align:center;margin:0 0 8px;}
.fc-workflow{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin:0 0 16px;}
.fc-workflow-step{
  position:relative;padding:12px 10px 12px 14px;background:#fff;
  border:1px solid var(--border-light);border-radius:var(--radius-sm);font-size:11px;color:#334155;
  text-align:left;min-height:72px;
}
.fc-workflow-step::before{
  content:attr(data-step);display:block;width:22px;height:22px;line-height:22px;
  text-align:center;border-radius:50%;background:var(--accent);color:#fff;
  font-size:11px;font-weight:700;margin-bottom:8px;
}
.fc-workflow-step strong{display:block;font-size:12px;color:var(--text);margin-bottom:2px;}
.fc-workflow-step .muted{display:block;font-size:10px;color:var(--muted);margin-top:2px;line-height:1.35;}
.fc-workflow-arrow{display:none;}
.fc-story-wrap{margin:0 0 16px;}
.fc-story-svg{display:block;min-height:200px;}
.fc-story-title{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.05em;
  color:var(--muted);text-align:center;margin:0 0 8px;}
.fc-bars-panel{background:#fff;border:1px solid var(--border-light);border-radius:var(--radius-sm);padding:14px 16px;}
.fc-bars-title{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.05em;color:var(--muted);margin:0 0 10px;}
.fc-bar-row{display:grid;grid-template-columns:115px 1fr 105px;gap:10px;align-items:center;margin-bottom:12px;font-size:12px;}
.fc-bar-row:last-child{margin-bottom:0;}
.fc-bar-label{color:#475569;font-weight:600;}
.fc-bar-track{position:relative;height:26px;background:#e2e8f0;border-radius:8px;overflow:hidden;}
.fc-bar-track::after{content:'';position:absolute;left:50%;top:4px;bottom:4px;width:2px;background:#94a3b8;z-index:1;border-radius:1px;}
.fc-bar-fill{position:absolute;top:4px;bottom:4px;border-radius:6px;z-index:0;transition:width .3s ease;}
.fc-bar-miss{background:linear-gradient(90deg,#fb923c,#ea580c);}
.fc-bar-corr{background:linear-gradient(90deg,#2dd4bf,var(--teal));}
.fc-bar-value{text-align:right;font-weight:700;font-size:11px;}
.fc-action-cards-title{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.06em;color:var(--muted);margin:0 0 10px;}
.fc-action-cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:12px;}
.exec-corr-cards{grid-template-columns:1fr 1fr;}
.fc-action-card{
  background:#fff;border:2px solid #99f6e4;border-radius:var(--radius-sm);padding:14px 12px;
  text-align:center;transition:transform .15s ease,box-shadow .15s ease;min-height:108px;
  display:flex;flex-direction:column;align-items:center;justify-content:center;
}
.fc-action-card:hover{transform:translateY(-2px);box-shadow:0 6px 16px rgba(15,23,42,.1);}
.fc-action-card-range{border-color:#fdba74;background:linear-gradient(180deg,#fff7ed 0%,#fff 100%);}
.fc-action-card-defl{border-color:#c4b5fd;background:linear-gradient(180deg,#f5f3ff 0%,#fff 100%);}
.fc-action-card-elev{border-color:#fcd34d;background:linear-gradient(180deg,#fefce8 0%,#fff 100%);}
.fc-action-card-map{border-color:#5eead4;background:linear-gradient(180deg,#ecfdf5 0%,#fff 100%);}
.fc-action-badge{
  display:inline-block;font-size:9px;font-weight:700;text-transform:uppercase;letter-spacing:.06em;
  padding:2px 8px;border-radius:999px;background:#0d9488;color:#fff;margin-bottom:8px;
}
.fc-action-arrow{font-size:1.75rem;line-height:1;color:var(--teal);font-weight:700;margin-bottom:4px;}
.fc-action-card-range .fc-action-arrow{color:#ea580c;}
.fc-action-card-defl .fc-action-arrow{color:#7c3aed;}
.fc-action-label{font-size:10px;text-transform:uppercase;letter-spacing:.05em;color:var(--muted);font-weight:600;}
.fc-action-value{
  display:flex;flex-wrap:wrap;align-items:baseline;justify-content:center;gap:4px;
  margin-top:6px;line-height:1.2;
}
.fc-action-dir{font-size:.85rem;font-weight:700;color:#475569;}
.fc-action-num{font-size:1.35rem;font-weight:800;color:var(--text);}
.fc-action-unit{font-size:.8rem;font-weight:600;color:var(--muted);}
.fc-action-sub{font-size:10px;color:var(--muted);margin-top:8px;}
.exec-split{
  display:grid;grid-template-columns:1fr auto 1.2fr;gap:12px;margin-top:18px;align-items:stretch;
}
.exec-split-bridge{
  display:flex;flex-direction:column;align-items:center;justify-content:center;
  padding:0 4px;min-width:52px;color:var(--muted);text-align:center;
}
.exec-bridge-arrow{
  display:flex;align-items:center;justify-content:center;width:36px;height:36px;
  border-radius:50%;background:#f1f5f9;border:2px solid var(--border);font-size:18px;
  font-weight:700;color:#64748b;margin-bottom:6px;
}
.exec-bridge-text{font-size:9px;font-weight:700;text-transform:uppercase;letter-spacing:.05em;line-height:1.3;max-width:56px;}
.exec-miss-panel,.exec-corr-panel{
  background:#fff;border:1px solid var(--border-light);border-radius:var(--radius-sm);
  overflow:hidden;box-shadow:var(--shadow-sm);display:flex;flex-direction:column;
}
.exec-miss-panel{border-top:3px solid #ea580c;}
.exec-corr-panel{border-top:3px solid #0d9488;}
.exec-panel-header{
  display:flex;align-items:center;gap:10px;padding:14px 16px;border-bottom:1px solid var(--border-light);
}
.exec-panel-header-miss{background:linear-gradient(180deg,#fff7ed 0%,#fff 100%);}
.exec-panel-header-corr{background:linear-gradient(180deg,#ecfdf5 0%,#fff 100%);}
.exec-panel-step{
  flex-shrink:0;width:22px;height:22px;border-radius:6px;font-size:11px;font-weight:800;
  display:flex;align-items:center;justify-content:center;line-height:1;
}
.exec-panel-header-miss .exec-panel-step{background:#ea580c;color:#fff;}
.exec-panel-header-corr .exec-panel-step{background:#0d9488;color:#fff;}
.exec-panel-icon{
  flex-shrink:0;width:32px;height:32px;border-radius:10px;font-size:15px;font-weight:800;
  display:flex;align-items:center;justify-content:center;line-height:1;
}
.exec-panel-header-miss .exec-panel-icon{background:#fed7aa;color:#c2410c;}
.exec-panel-header-corr .exec-panel-icon{background:#99f6e4;color:#047857;}
.exec-panel-text{flex:1;min-width:0;}
.exec-panel-title{display:block;font-size:14px;color:var(--text);font-weight:700;line-height:1.25;}
.exec-panel-sub{display:block;font-size:11px;color:var(--muted);margin-top:3px;line-height:1.35;}
.exec-panel-body{padding:14px 16px 16px;flex:1;}
.exec-compass-wrap{background:#fafbfc;border:1px solid var(--border-light);border-radius:8px;padding:6px;}
.exec-visual-caption{font-size:11px;color:#92400e;margin:8px 0 0;text-align:center;line-height:1.4;}
.exec-miss-list{list-style:none;margin:12px 0 0;padding:0;display:flex;flex-wrap:wrap;gap:8px;}
.exec-miss-list li{
  flex:1 1 auto;min-width:90px;padding:8px 10px;background:#fff7ed;border:1px solid #fed7aa;
  border-radius:8px;font-size:11px;color:#9a3412;text-align:center;
}
.exec-miss-list li strong{display:block;font-size:1.1rem;color:#c2410c;font-weight:800;}
.exec-corr-panel .fc-action-cards{margin:0;padding:0 2px;}
.exec-legend-note{
  margin:12px 0 0;padding:10px 12px;background:#f8fafc;border-radius:8px;font-size:11px;color:var(--muted);line-height:1.45;
}
.exec-legend-note .legend-miss{color:#ea580c;font-weight:600;}
.exec-legend-note .legend-corr{color:#0d9488;font-weight:600;}
.exec-story-lead{margin:0 0 14px;padding:14px 16px;background:rgba(255,255,255,.7);
  border:1px solid #a7f3d0;border-radius:var(--radius-sm);}
.exec-story-lead p{margin:0;font-size:.98rem;color:#134e4a;line-height:1.55;}
.exec-story-lead p+p{margin-top:8px;}
.exec-story-lead .exec-big{font-size:1.35rem;font-weight:800;color:#065f46;}
.exec-visual-row{display:grid;grid-template-columns:minmax(220px,300px) 1fr;gap:18px;align-items:start;margin-top:16px;}
.report-collapsible{margin-top:14px;border:1px solid var(--border-light);border-radius:var(--radius-sm);background:#fafbfc;}
.report-collapsible summary{
  cursor:pointer;padding:14px 16px;font-weight:600;font-size:13px;color:#475569;
  list-style:none;user-select:none;transition:background .15s;
}
.report-collapsible summary:hover{background:#f1f5f9;}
.report-collapsible summary::-webkit-details-marker{display:none;}
.report-collapsible summary::before{content:'▸ ';color:var(--muted);}
.report-collapsible[open] summary::before{content:'▾ ';}
.report-collapsible .report-collapsible-body{padding:0 16px 16px;}
.report-executive{
  background:linear-gradient(165deg,#ecfdf5 0%,#fff 45%,#f0fdf4 100%);
  border:1px solid #6ee7b7;border-radius:var(--radius);padding:0;margin-bottom:18px;
  box-shadow:var(--shadow-sm);overflow:hidden;
}
.report-executive-head{padding:20px 22px 16px;border-bottom:1px solid #a7f3d0;
  background:linear-gradient(90deg,rgba(16,185,129,.08) 0%,transparent 100%);}
.report-executive h2{margin:0;font-size:1.15rem;color:#065f46;font-weight:800;}
.report-executive-badge{
  display:inline-block;margin-left:10px;padding:3px 10px;font-size:10px;font-weight:700;
  text-transform:uppercase;letter-spacing:.06em;background:#10b981;color:#fff;border-radius:999px;vertical-align:middle;
}
.report-executive-body{padding:18px 22px 22px;}
.report-executive-lead{font-size:1.05rem;line-height:1.6;color:#134e4a;margin:0 0 12px;}
.plain-list{margin:0;padding-left:1.25rem;font-size:.95rem;line-height:1.65;}
.plain-list li{margin-bottom:8px;}
.plain-list strong{color:var(--text);}
.next-round-box{
  background:#fff;border:1px solid #a7f3d0;border-radius:var(--radius-sm);
  padding:14px 16px;margin-top:16px;
}
.next-round-box h4{margin:0 0 10px;font-size:.8rem;color:#047857;text-transform:uppercase;letter-spacing:.05em;font-weight:700;}
.reading-guide{
  background:#fff;border:1px solid var(--border-light);border-radius:var(--radius);
  margin-bottom:18px;box-shadow:var(--shadow-sm);overflow:hidden;
}
.reading-guide-head{padding:16px 20px;background:linear-gradient(90deg,#f0f9ff 0%,#fff 100%);
  border-bottom:1px solid var(--border-light);}
.reading-guide h3{margin:0;font-size:1rem;color:#0369a1;font-weight:800;}
.reading-guide-intro{margin:6px 0 0;font-size:13px;color:var(--muted);line-height:1.45;}
.guide-flow{
  display:flex;flex-wrap:wrap;align-items:center;gap:6px 4px;padding:14px 20px;
  background:#f8fafc;border-bottom:1px solid var(--border-light);
}
.guide-flow-label{
  font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.06em;
  color:var(--muted);margin-right:8px;
}
.guide-flow-step{
  display:inline-flex;align-items:center;gap:6px;padding:6px 12px;border-radius:999px;
  background:#fff;border:1px solid var(--border);font-size:12px;font-weight:600;
  color:var(--text);text-decoration:none;transition:all .15s ease;
}
.guide-flow-step:hover{background:var(--accent);border-color:var(--accent);color:#fff;}
.guide-flow-step:hover .guide-flow-num{background:rgba(255,255,255,.25);color:#fff;}
.guide-flow-num{
  width:20px;height:20px;border-radius:50%;background:#e2e8f0;color:#475569;
  font-size:11px;font-weight:800;display:flex;align-items:center;justify-content:center;line-height:1;
}
.guide-flow-arrow{color:#94a3b8;font-size:12px;font-weight:700;padding:0 2px;}
.guide-cards{
  display:grid;grid-template-columns:repeat(3,1fr);gap:12px;padding:16px 20px 20px;
}
.guide-card{
  display:flex;flex-direction:column;gap:10px;padding:14px 14px 12px;
  background:#fff;border:1px solid var(--border-light);border-radius:var(--radius-sm);
  text-decoration:none;color:inherit;transition:transform .15s ease,box-shadow .15s ease,border-color .15s;
  min-height:120px;
}
.guide-card:hover{
  transform:translateY(-2px);box-shadow:0 8px 20px rgba(15,23,42,.08);
  border-color:#cbd5e1;
}
.guide-card-top{display:flex;align-items:flex-start;gap:10px;}
.guide-card-icon{
  flex-shrink:0;width:40px;height:40px;border-radius:10px;display:flex;
  align-items:center;justify-content:center;font-size:18px;line-height:1;
}
.guide-card-icon svg{width:28px;height:28px;display:block;}
.guide-card--summary .guide-card-icon{background:#ecfdf5;color:#047857;border:1px solid #a7f3d0;}
.guide-card--story .guide-card-icon{background:#fff7ed;color:#c2410c;border:1px solid #fed7aa;}
.guide-card--diagrams .guide-card-icon{background:#eff6ff;color:#2563eb;border:1px solid #bfdbfe;}
.guide-card--tables .guide-card-icon{background:#f1f5f9;color:#475569;border:1px solid #e2e8f0;}
.guide-card--map .guide-card-icon{background:#f5f3ff;color:#7c3aed;border:1px solid #ddd6fe;}
.guide-card--nav .guide-card-icon{background:#f0f9ff;color:#0369a1;border:1px solid #bae6fd;}
.guide-card-title{display:block;font-size:13px;font-weight:700;color:var(--text);line-height:1.25;}
.guide-card-desc{display:block;font-size:11px;color:var(--muted);line-height:1.4;margin-top:2px;}
.guide-card-link{
  font-size:10px;font-weight:700;color:var(--accent);margin-top:auto;
  text-transform:uppercase;letter-spacing:.04em;
}
.guide-card:hover .guide-card-link{color:#1d4ed8;}
.guide-preview{
  height:36px;border-radius:6px;background:#fafbfc;border:1px dashed var(--border);
  display:flex;align-items:center;justify-content:center;gap:4px;padding:4px 8px;
}
.guide-preview-dot{width:8px;height:8px;border-radius:50%;}
.guide-preview-bar{height:6px;border-radius:3px;background:linear-gradient(90deg,#fb923c,#ea580c);}
.guide-preview-bar-corr{background:linear-gradient(90deg,#2dd4bf,#0d9488);}
@media (max-width:900px){.guide-cards{grid-template-columns:repeat(2,1fr);}}
@media (max-width:560px){.guide-cards{grid-template-columns:1fr;}.guide-flow{justify-content:flex-start;}}
.report-glossary details{border:1px solid var(--border-light);border-radius:var(--radius-sm);padding:12px 16px;background:#fafbfc;}
.report-glossary summary{cursor:pointer;font-weight:600;color:#475569;font-size:.9rem;}
.report-glossary dl{margin:12px 0 0;font-size:.85rem;}
.report-glossary dt{font-weight:600;color:#334155;margin-top:10px;}
.report-glossary dd{margin:2px 0 0;color:var(--muted);}
.report-footer{
  margin-top:28px;padding:20px 24px;text-align:center;font-size:12px;color:var(--muted);
  border-top:1px solid var(--border-light);background:#fff;border-radius:var(--radius);
}
.report-footer strong{color:var(--text);}
.back-to-top{
  position:fixed;bottom:24px;right:24px;width:44px;height:44px;border-radius:50%;
  background:var(--accent);color:#fff;border:none;cursor:pointer;font-size:18px;
  box-shadow:0 4px 14px rgba(37,99,235,.4);opacity:0;visibility:hidden;
  transition:opacity .2s,visibility .2s,transform .2s;z-index:300;
}
.back-to-top.visible{opacity:1;visibility:visible;}
.back-to-top:hover{transform:translateY(-2px);}
@media (max-width:800px){
  .fc-diagram-grid,.fc-diagram-grid-4,.fc-workflow{grid-template-columns:1fr;}
  .exec-visual-row,.exec-split{grid-template-columns:1fr;}
  .exec-split-bridge{
    flex-direction:row;gap:8px;padding:8px 0;min-width:0;justify-content:center;
  }
  .exec-bridge-arrow{margin:0;}
  .exec-bridge-text{max-width:none;}
  .exec-corr-cards{grid-template-columns:1fr 1fr;}
  .report-header-inner{flex-direction:column;}
  .hero-kpis{width:100%;}
  .fc-bar-row{grid-template-columns:1fr;gap:4px;}
  .fc-bar-value{text-align:left;}
}
@media print{
  .report-nav-wrap,.back-to-top{display:none!important;}
  body{background:#fff;}
  .report-page{padding:0;max-width:none;}
  .section-card,.report-executive{box-shadow:none;break-inside:avoid;}
  .report-collapsible,.report-glossary details{border:none;}
  .report-collapsible .report-collapsible-body{display:block!important;}
  details summary{list-style:none;}
}
"""

REPORT_SCRIPT = """
(function(){
  var nav=document.querySelector('.report-nav');
  var links=nav?nav.querySelectorAll('a[href^=\"#\"]'):[];
  var sections=[];
  links.forEach(function(a){
    var id=a.getAttribute('href').slice(1);
    var el=document.getElementById(id);
    if(el)sections.push({link:a,el:el});
  });
  function onScroll(){
    var y=window.scrollY+120;
    var current=null;
    sections.forEach(function(s){
      if(s.el.offsetTop<=y)current=s.link;
    });
    links.forEach(function(a){a.classList.remove('is-active');});
    if(current)current.classList.add('is-active');
    var btn=document.getElementById('back-to-top');
    if(btn)btn.classList.toggle('visible',window.scrollY>400);
  }
  window.addEventListener('scroll',onScroll,{passive:true});
  onScroll();
  var topBtn=document.getElementById('back-to-top');
  if(topBtn)topBtn.addEventListener('click',function(){window.scrollTo({top:0,behavior:'smooth'});});
})();
"""
