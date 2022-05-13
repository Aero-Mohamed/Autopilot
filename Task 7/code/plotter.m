titles = {"Beta (deg)", "P (deg)", "r (deg)", "phi (deg)", "psi (deg)"};
for n=1:5
   subplot(5, 1, n);
   plot(out.simout.Time, out.simout.Data(:, n), 'LineWidth', 3);
   title(titles{n});
end